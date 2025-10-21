# Simple blink example

import time
from machine import Pin

led = Pin(25, Pin.OUT)
enable = Pin(29, Pin.OUT)
enable.value(1)  # power on the sensor
trigger = Pin(28, Pin.IN, Pin.PULL_DOWN)

flag = 0

while True:
    if(trigger.value() == 1 or flag == 1):
        flag = 1
        led.value(0)  # yellow LED on
        time.sleep_ms(1000)
        led.value(1)  # yellow LED off
        time.sleep_ms(1000)

