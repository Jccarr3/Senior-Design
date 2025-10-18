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


# #########################################################################################
# #Peripheral code for bluetooth connection with PICOs

# import sys
# from micropython import const       #used to efficientyl manage memory
# import bluetooth                    #self explanatory
# import asyncio                      #used for asynchronous programming
# import aioble                       #aioble library for bluetooth low energy

# import struct                       #used for converting between python values and C structs

# #custom 128-bit UUIDs for ZUMO service and characteristic
# _PICO_SERVICE_UUID = bluetooth.UUID("00001523-1212-EFDE-1523-785FEABCD123")
# _PICO_CHAR_UUID = bluetooth.UUID("00001524-1212-EFDE-1523-785FEABCD123")
# #custum 128-bit UUIDs for ZUMO service and characteristic

# #define custom service and characteristic
# service = aioble.Service(_PICO_SERVICE_UUID)
# int_char = aioble.Characteristic(service,_PICO_CHAR_UUID,read = True,notify = True)
# aioble.register_services(service)
# #define custom service and characteristic

# async def main():
#     await aioble.advertise(30000, name="PICO-peripheral", services=[_PICO_SERVICE_UUID])
#     print("Advertising PICO")

#     value = 0
#     while True:
#         value += 1
#         int_char.write(struct.pack("<i",value))
#         print("sent value:", value)
#         await asyncio.sleep(1)

# asyncio.run(main())
# #end of peripheral code for bluetooth connection with PICOs
# ########################################################################################

########################################################################################
#code for central device to connect to PICOs
import sys
from micropython import const       #used to efficientyl manage memory
import bluetooth                    #self explanatory
import asyncio                      #used for asynchronous programming
import aioble                       #aioble library for bluetooth low energy

import struct                       #used for converting between python values and C structs

#custom 128-bit UUIDs for ZUMO service and characteristic
_PICO_SERVICE_UUID = bluetooth.UUID("00001523-1212-EFDE-1523-785FEABCD123")
_PICO_CHAR_UUID = bluetooth.UUID("00001524-1212-EFDE-1523-785FEABCD123")
#end of custum 128-bit UUIDs for ZUMO service and characteristic

#code to decode data from peripheral
def decode_data(data):
    return struct.unpack("<i",data)[0]
#end of code to decode data from peripheral

#code to scan for PICO device
async def find_pico_device():
    async with aioble.scan(5000, interval_us=30000,window_us=30000,active=True) as scanner:
        async for result in scanner:
            if result.name() == "PICO-Peripheral" and _PICO_SERVICE_UUID in result.services():
                return result.device
            
        return None
#end of code to scan for PICO device

#main loop code
async def main():
    device = await find_pico_device()
    if not device:
        print("PICO not found")
        return
    
    try:
        print("Connecting to", device)
        connection = await device.connect()
    except asyncio.TimeoutError:
        print("Timout connecting to device")
        return
    
    async with connection:
        try:
            pico_service = await connection.service(_PICO_SERVICE_UUID)
            pico_characteristic = await pico_service.characteristic(_PICO_CHAR_UUID)
        except asyncio.TimeoutError:
            print("Timout discovering services/characteristics")
            return
        while connection.is_connected():
            data = decode_data(await pico_characteristic.read())
            print("Received data: ", data)
            await asyncio.sleep(1)
#end of main loop code

asyncio.run(main())
#end of code for central device to connect to PICOs
########################################################################################
