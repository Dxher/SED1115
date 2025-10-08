from machine import Pin, PWM, ADC
import time
adcA = ADC(Pin(26))
led1 = PWM(Pin(18))
led1.freq(1000)
# This could also be written as: led1 = PWM(Pin(18), freq=1000)
while True:
    value = adcA.read_u16()
    print(value)
    led1.duty_u16(value)
    time.sleep_ms(50) 