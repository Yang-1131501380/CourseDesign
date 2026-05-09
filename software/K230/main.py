"""
K230 red target tracking with UART2 output.

Tracking uses red blob detection.
Distance uses a calibrated monocular approximation based on blob area, which is
more rotation-tolerant than using only the axis-aligned bounding box.
"""

import gc
import math
import os
import time

from machine import FPIOA
from machine import UART
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor

LCD_WIDTH = 800
LCD_HEIGHT = 480

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

UART_BAUDRATE = 256000

RED_THRESHOLDS = [(15, 78, 24, 89, 12, 92)]
AREA_THRESHOLD = 400
INFO_FONT_BIG = 48
INFO_FONT_MID = 34
INFO_TOP_Y = 6

TARGET_WIDTH_CM = 6.0
TARGET_HEIGHT_CM = 6.0

CAMERA_MATRIX = [
    740.7833044413, 0.0, 395.3743649561,
    0.0, 726.1232945757, 286.0559645531,
    0.0, 0.0, 1.0
]

DIST_COEFFS = [
    0.1020262474,
    0.1664580085,
    0.0008287791,
    -0.0590073729,
    -1.5538687919
]

try:
    from calibration_params import CAMERA_MATRIX as FILE_CAMERA_MATRIX
    from calibration_params import DIST_COEFFS as FILE_DIST_COEFFS

    CAMERA_MATRIX = FILE_CAMERA_MATRIX
    DIST_COEFFS = FILE_DIST_COEFFS
    print("use calibration_params.py")
except ImportError:
    print("use built-in calibration params")

FX = CAMERA_MATRIX[0]
FY = CAMERA_MATRIX[4]
FOCAL_MEAN = math.sqrt(FX * FY)


def uart_init():
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    return UART(UART.UART2, UART_BAUDRATE)


def sensor_init():
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor


def checksum_xor(payload):
    checksum = 0

    for ch in payload:
        checksum ^= ord(ch)

    return checksum


def send_packet(uart, valid, cx, cy, dx, dy, distance_cm, width, height):
    payload = "K230,{},{},{},{},{},{:.2f},{},{}".format(
        valid, cx, cy, dx, dy, distance_cm, width, height
    )
    checksum = checksum_xor(payload)
    packet = "${}*{:02X}\r\n".format(payload, checksum)
    uart.write(packet)
    print(packet, end="")


def score_blob(blob):
    return blob.pixels()


def select_best_blob(blobs):
    if not blobs:
        return None

    best_blob = blobs[0]
    best_score = score_blob(best_blob)

    for blob in blobs[1:]:
        blob_score = score_blob(blob)

        if blob_score > best_score:
            best_blob = blob
            best_score = blob_score

    return best_blob


def clamp_positive(value, default_value):
    if value > 1.0:
        return value

    return default_value


def estimate_distance_cm(blob):
    width_px = clamp_positive(float(blob.w()), 1.0)
    height_px = clamp_positive(float(blob.h()), 1.0)
    area_px = clamp_positive(float(blob.pixels()), width_px * height_px)

    dist_from_width = FX * TARGET_WIDTH_CM / width_px
    dist_from_height = FY * TARGET_HEIGHT_CM / height_px
    dist_from_area = FOCAL_MEAN * math.sqrt(
        TARGET_WIDTH_CM * TARGET_HEIGHT_CM / area_px
    )

    estimates = [dist_from_width, dist_from_height, dist_from_area]
    estimates.sort()
    median_estimate = estimates[1]

    # Area estimate is usually less sensitive to in-plane rotation for a
    # uniformly colored square target, while the median suppresses outliers.
    return (dist_from_area * 0.7) + (median_estimate * 0.3)


def correct_distance_cm(raw_cm):
    fixed_cm = 0.7 * raw_cm + 2.45
    return fixed_cm


def draw_center_text(img, y, font_size, text, color):
    text_width = len(text) * font_size // 2
    x = max(0, (IMAGE_WIDTH - text_width) // 2)

    img.draw_string_advanced(x + 2, y + 2, font_size, text,
                             color=(0, 0, 0))
    img.draw_string_advanced(x, y, font_size, text, color=color)


def draw_status_text(img, valid, distance_cm, dx, dy, fps_value):
    if valid:
        draw_center_text(
            img,
            INFO_TOP_Y,
            INFO_FONT_BIG,
            "D:{:.1f}cm".format(distance_cm),
            (255, 255, 0)
        )
        draw_center_text(
            img,
            INFO_TOP_Y + INFO_FONT_BIG + 4,
            INFO_FONT_MID,
            "dx:{} dy:{} fps:{:.1f}".format(dx, dy, fps_value),
            (255, 255, 255)
        )
    else:
        draw_center_text(
            img,
            INFO_TOP_Y,
            INFO_FONT_BIG,
            "TARGET LOST",
            (255, 64, 64)
        )
        draw_center_text(
            img,
            INFO_TOP_Y + INFO_FONT_BIG + 4,
            INFO_FONT_MID,
            "fps:{:.1f}".format(fps_value),
            (255, 255, 255)
        )


def draw_target(img, blob, distance_cm, fps_value):
    x = blob.x()
    y = blob.y()
    w = blob.w()
    h = blob.h()
    cx = blob.cx()
    cy = blob.cy()
    dx = int(cx - IMAGE_WIDTH / 2)
    dy = int(cy - IMAGE_HEIGHT / 2)

    img.draw_rectangle(x, y, w, h, color=(255, 0, 0), thickness=2)
    img.draw_cross(cx, cy, color=(0, 255, 0), thickness=2)
    img.draw_cross(IMAGE_WIDTH // 2, IMAGE_HEIGHT // 2,
                   color=(255, 255, 255), thickness=1)
    draw_status_text(img, True, distance_cm, dx, dy, fps_value)


def draw_no_target(img, fps_value):
    img.draw_cross(IMAGE_WIDTH // 2, IMAGE_HEIGHT // 2,
                   color=(255, 255, 255), thickness=1)
    draw_status_text(img, False, 0.0, 0, 0, fps_value)


def main():
    uart = uart_init()
    sensor = sensor_init()
    clock = time.clock()

    Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT,
                 to_ide=True, quality=50)
    MediaManager.init()
    sensor.run()
    time.sleep_ms(200)

    try:
        while True:
            clock.tick()
            img = sensor.snapshot()
            blobs = img.find_blobs(
                RED_THRESHOLDS,
                area_threshold=AREA_THRESHOLD
            )
            best_blob = select_best_blob(blobs)
            fps_value = clock.fps()

            if best_blob is not None:
                cx = int(best_blob.cx())
                cy = int(best_blob.cy())
                dx = int(cx - IMAGE_WIDTH / 2)
                dy = int(cy - IMAGE_HEIGHT / 2)
                raw_distance_cm = estimate_distance_cm(best_blob)
                distance_cm = correct_distance_cm(raw_distance_cm)

                send_packet(
                    uart,
                    1,
                    cx,
                    cy,
                    dx,
                    dy,
                    distance_cm,
                    best_blob.w(),
                    best_blob.h()
                )
                draw_target(img, best_blob, distance_cm, fps_value)
            else:
                send_packet(uart, 0, 0, 0, 0, 0, 0.0, 0, 0)
                draw_no_target(img, fps_value)

            Display.show_image(img)
            gc.collect()
    finally:
        sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


main()
