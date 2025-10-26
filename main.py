# Senior Design (ECE 484): Team 37
# Version: Competition 1 (October 28th, 2025)
# Unenhanced Zumo 2040 Competition Software: main.py
# 
# At the start of competition the robot must go through a startup sequence:
#   1. Initialize, if any, some or all of the Zumo 2040 peripherals. Identify what peripherals need to be initialized, how to do so, and why it's needed. Peripheral initialization, if required, will likely depend on the functionalitiy and information desired from the peripheral to fulfill the intended software strategy.
#   2. Load the unique team identifier on the robot. This will likely make use of some or all of the sensory output peripherals: display, LEDs, buzzer.
#   3. Check for either one of the start triggers:
#           a) The Raspberry Pi Picos established bluetooth connection, which can be done in parallel to the Zumo 2040 Boot Up Process (albeit the Boot Up process will be stalled and waiting for the start trigger after team identification), causing the corresponding GPIO input trigger pins to read a logical one.
#           b) The robot's designated trigger button was pressed, causing a start flag to be set.  
#   4. Randomly select a beginning match strategy from the set of start strategies. A more advanced version of this feature can take advantage of the robot's flash storage to save information pertaining to the start of previous rounds, which in turn can be used to narrow down the selection of start strategies. This would involve saving key start 
#      parameters during the beginning of a round so they can be used in a strategy selection algorithim or even machine learning model. Note, the first round of any match will always have no beginning parameters to utilize, therefore, the algorithim/model must adapt its randomization based on the level of known information or attempt to make
#      broader more complex predictions. This could even involve trying to guess the opposing team and their start strategy based on a saved history, although this may prove too unreliable and resource intensive (i.e. too much memory and computation). 
#           - A random/pseudo-random number generator generates an index from a limited set of N indices, where N represents the total number of available start strategies, which is used as the selection mechanism to assign values to the machines state controller parameters. For example, index X could correspond to start strategy X which involves
#             a successive charge and retreat pattern until target is found or a collision from the opposing robot is detected. By assigning state machine parameters, control of the robot is centralized to the overall robot state controller which should improve efficiency and reliability of the code through a modular and distinct design approach. 
#             However, the controller parameters must be numerous and varied enough to successfully support a variety of start strategies.
#
# After the Boot Up process, the robot enters into the central control state machine, where it uses control signals generated from the processed peripheral data streams. The interdependent control architecture governs how peripheral-generated control signals are handled, while those same signals can modify/change the controllers state. As such,
# the controller architecture utilizes output signal generation dependent on the state of the controller. This architecture is a mealy software state machine. The general software process below illustrates this point:
#   1. Control signals are generated from processed peripheral data streams. 
#   2. The control signals, along with the current state (i.e. the current behavior of the motors), are the input of a state generation algorithim / ML model. This will set the state parameter value of the controller (i.e. set the motor outputs).
#   NOTE #1: While the main system output is the behavior of the motors, control signals may communicate the current internal behavior of the machine by controlling the sensory output peripherals. However, this does not affect the state generation as this is just the UI representation of control signals. 
#   NOTE #2: This is just a preliminary software based controller and it's actual implementation is subject to change in this and other versions of competiton software. 


from zumo_2040_robot import robot
import time
from machine import Pin

#important variables/objects
led = Pin(25, Pin.OUT)
trigger = Pin(28, Pin.IN, Pin.PULL_DOWN)
flag = 0
state = "REST"
prev_time = 0                                #used for letting controller know when to check stuff

display = robot.Display()

rgbs = robot.RGBLEDs()
rgbs.set_brightness(3)

button_a = robot.ButtonA()
#important variables/objects


#function for setting up NC STATE identifier
def show_NCSTATE():
   flip = 0
   tuffy = display.load_pbm("zumo_2040_robot/extras/tuffy.pbm")
   display.blit(tuffy, 0, 0)

   for led in range(6):
      if (flip) == 0:
         rgbs.set(led,[255,0,0])
      else:
         rgbs.set(led,[200,200,200])

      if(led == 2): flip = not flip
      flip = not flip

   rgbs.show()
   display.show()
#function for setting up NC STATE identifier


#setup team identifiers
show_NCSTATE()
#setup team identifier





#main code execution
while True:
   #polling for button input/trigger
   if time.ticks_diff(time.ticks_ms(), prev_time) > 50:
      prev_time = time.ticks_ms()

      if button_a.is_pressed():                       #check for start signal from button A
         time.sleep(3)
         display.fill(0)
         tuffy = display.load_pbm("zumo_2040_robot/extras/signal_received.pbm")
         display.blit(tuffy, 0, 0)
         display.show()


      