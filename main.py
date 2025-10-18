# from machine import Pin
# from utime import sleep

# pin = Pin("LED", Pin.OUT)

# print("LED starts flashing...")
# while True:
#     try:
#         pin.value(not pin.value())
#         sleep(1) # sleep 1sec
#     except KeyboardInterrupt:
#         break
# pin.off()
# print("Finished.")



#code for bluetooth connection with PICOs

import sys                                  #this import command is used to access system-specific parameters and functions
import aioble                              #this will be used for a BLE implementation
import bluetooth                            #self explanatory
import asyncio                              #this import command is used to work with asynchronous programming in MicroPython
import struct                               #this import command is used to work with binary data and convert between Python values and C structs


# Define UUIDs for service and characteristic
_SERVICE_UUID = bluetooth.UUID(0x1848)         # Define a custom service UUID
_CHAR_UUID = bluetooth.UUID(0x2A6E)            # Define a custom characteristic UUID
# Define UUIDs for service and characteristic

# Bluetooth Parameters
BLE_NAME = "Pico Central"
BLE_SVC_UUID = bluetooth.UUID(0x181A)                       # Environmental Sensing Service
BLE_CHARACTERISTIC_UUID = bluetooth.UUID(0x2A6E)            # Temperature Characteristic
BLE_APPERANCE = 0x0300                                      # Generic Tag
BLE_ADVERTISING_INTERVAL = 2000                             # Advertising interval in milliseconds
BLE_SCAN_DURATION = 5000                                    # Scan duration in milliseconds
BLE_INTERVAL = 30000                                        # Interval between scans in milliseconds
BLE_WINDOW = 30000                                          # Scan window in milliseconds



    