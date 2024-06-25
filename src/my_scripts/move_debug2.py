#!/usr/bin/env python3
# 06/25/2025 initial to test comm to M1 and M2 vescs

import time
import lgpio

#define motor pins
M1_ADC = 13
M1_DIR = 6
M2_ADC = 12
M2_DIR = 16

#define PWM frequency
FREQ_M1 = 585
FREQ_M2 = 585

#open gpio and set digital pins as output
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h,M1_DIR)
lgpio.gpio_claim_output(h,M2_DIR)


def motorStop():#Motor stops

    lgpio.tx_pwm(h,M1_ADC, FREQ_M1, 0)
    lgpio.gpio_write(h,M1_ADC,0)
    lgpio.tx_pwm(h,M2_ADC, FREQ_M2, 0)
    lgpio.gpio_write(h,M2_ADC,0)


if __name__ == '__main__':
    try:
        while True:
            print("hello")
            speed_setM1 = 25
            speed_setM2 = 30
            lgpio.gpio_write(h,M1_DIR,1)
            lgpio.tx_pwm(h,M1_ADC,FREQ_M1,speed_setM1)
            lgpio.gpio_write(h,M2_DIR,1)
            lgpio.tx_pwm(h,M2_ADC,FREQ_M2,speed_setM2)

    except KeyboardInterrupt:
            motorStop()
            lgpio.gpiochip_close(h)






