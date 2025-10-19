# Simple blink example

import time
from machine import Pin

led = Pin(25, Pin.OUT)
v_out = Pin(28, Pin.OUT)
trigger = Pin(25, Pin.IN)

v_out.value(1)  # power pico board
while True:
    if(trigger.value() == 1):
        led.value(0)  # yellow LED on
        time.sleep_ms(1000)
        led.value(1)  # yellow LED off
        time.sleep_ms(1000)

