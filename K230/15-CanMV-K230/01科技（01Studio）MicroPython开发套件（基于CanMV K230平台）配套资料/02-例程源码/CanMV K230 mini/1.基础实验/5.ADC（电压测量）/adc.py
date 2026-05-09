'''
实验名称：ADC（电压测量）
版本：v1.0
作者：01Studio
实验平台：01Studio CanMV K230 mini
说明：ADC共4个通道，CanMV K230 mini只引出其中通道0、1，位于背面焊盘需要自行焊接。实际量程为0-1.8V。
    （请勿超出测量量程, 可能导致主控芯片烧坏！）
'''

from machine import ADC
import time


#构建ADC对象:
adc = ADC(0) #通道0

while True:

    print(adc.read_u16()) # 获取ADC通道采样值

    # 获取ADC通道电压值，保留2为小数。通道0、1实际量程为0-1.8V。
    print('%.2f'%(adc.read_uv()/1000000), "V")

    time.sleep(1) #延时1秒
