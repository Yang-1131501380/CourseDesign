"""
K230 red target tracking with UART2 output.

Tracking uses LAB red blob detection. Distance uses a calibrated monocular
approximation based on blob area, which is more tolerant of target rotation
than strict rectangle PnP for a plain red block.
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


LCD_WIDTH = 640
LCD_HEIGHT = 480

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

UART_BAUDRATE = 256000

RED_THRESHOLDS = [(30, 100, 15, 127, 15, 127)]
AREA_THRESHOLD = 400

TARGET_WIDTH_CM = 6.0
TARGET_HEIGHT_CM = 6.0
TARGET_AREA_CM2 = TARGET_WIDTH_CM * TARGET_HEIGHT_CM

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
    center_error = abs(blob.cx() - IMAGE_WIDTH / 2) + \
        abs(blob.cy() - IMAGE_HEIGHT / 2)
    area = blob.pixels()
    return area - center_error * 2


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


def median3(a, b, c):
    if a > b:
        a, b = b, a
    if b > c:
        b, c = c, b
    if a > b:
        a, b = b, a

    return b


def estimate_distance_cm(blob):
    width_px = max(blob.w(), 1)
    height_px = max(blob.h(), 1)
    area_px = max(blob.pixels(), 1)

    dist_from_width = FX * TARGET_WIDTH_CM / width_px
    dist_from_height = FY * TARGET_HEIGHT_CM / height_px
    dist_from_area = FOCAL_MEAN * math.sqrt(TARGET_AREA_CM2 / area_px)
    median_estimate = median3(dist_from_width, dist_from_height,
                              dist_from_area)

    return (dist_from_area * 0.7) + (median_estimate * 0.3)


def correct_distance_cm(raw_cm):
    fixed_cm = 0.736 * raw_cm + 2.45
    print("raw:", raw_cm, "fixed:", fixed_cm)
    return fixed_cm


def draw_target(img, blob, distance_cm, fps_value):
    cx = int(blob.cx())
    cy = int(blob.cy())
    dx = int(cx - IMAGE_WIDTH / 2)
    dy = int(cy - IMAGE_HEIGHT / 2)

    img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
    img.draw_cross(cx, cy, color=(0, 255, 0), thickness=2)
    img.draw_string_advanced(
        0, 0, 24,
        "dx:{} dy:{} {:.1f}cm fps:{:.1f}".format(
            dx, dy, distance_cm, fps_value
        ),
        color=(255, 255, 255)
    )


def draw_no_target(img, fps_value):
    img.draw_string_advanced(
        0, 0, 24,
        "target lost fps:{:.1f}".format(fps_value),
        color=(255, 255, 255)
    )


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
                area_threshold=AREA_THRESHOLD,
                merge=True
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
