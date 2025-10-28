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

#JORDAN COMMENTS!!!!!!!!!!!!!!!
#  Charge State:
#     - While charging at robot itilize PID control system to make constant adjustments while charging.
#     - When close enoughh to opponent robot, set state to "PIT" which will circle the robot and hit its side
from zumo_2040_robot import robot
import time
from machine import Pin
import random

#important variables/objects
led = Pin(25, Pin.OUT)
trigger = Pin(28, Pin.IN, Pin.PULL_DOWN)
flag = 0
left_retreat_flag , right_retreat_flag = 0, 0      #flags used to set proper retreat state
state = "START"
prev_time = 0                                #used for letting controller know when to check stuff

   #display
display = robot.Display()

   #leds
rgbs = robot.RGBLEDs()
rgbs.set_brightness(3)

   #buttons
button_a = robot.ButtonA()

   #line sensors
line_sensors = robot.LineSensors()

   #proximity sensors
proximity_sensors = robot.ProximitySensors()

   #motors
motors = robot.Motors()

#important variables/objects


#function for setting up NC STATE identifier
def show_NCSTATE():
   flip = 0
   tuffy = display.load_pbm("zumo_display/tuffy.pbm")
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

#line detection function
def floor_scan():
   global left_retreat_flag, right_retreat_flag, state
   line_sensors.start_read(emitters_on = True)
   time.sleep_ms(2)

   line = line_sensors.read()
   for i in range(5):
      if((line[i] < 500) and (i <= 1)):
         state = "RECOVER"
         right_retreat_flag = 1
         break
      elif((line[i] < 500) and (i > 2)):
         left_retreat_flag = 1
         state = "RECOVER"
         break
         
#line detection function

#proximity detection function
def proximity_scan():
   proximity_sensors.read()

   readings = [proximity_sensors.left_counts_with_left_leds(),]
   #set global for which sensors detected enemy

#proximity detection function

#ALL FUNCTIONS INVOLVING MOTORS
   #right recovery function(used for recovering from edge detection)
def right_recovery():
   motors.set_speeds(-1*5800,-1*1800)
   time.sleep_ms(750)
   motors.off()
   #right recovery function(used for recovering from edge detection)

   #left recovery function(used for recovering from edge detection)
def left_recovery():
   motors.set_speeds(-1*1800,-1*5800)
   time.sleep_ms(750)
   motors.off()
   #left recovery function(used for recovering from edge detection)

   #Question mark shape(used as a startup option to get behind opponent)
def question_mark_kick():
   motors.set_speeds(-1*1800,-1*5800)
   time.sleep_ms(1100)
   motors.set_speeds(-1*5800,-1*1800)
   time.sleep_ms(300)
   motors.off()
   #Question mark shape(used as a startup option to get behind opponent

#main code execution
while True:
   #polling for button input/trigger
   if flag == 0:
      if time.ticks_diff(time.ticks_ms(), prev_time) > 50:
         prev_time = time.ticks_ms()
         if button_a.is_pressed():                       #check for start signal from button A
            line_sensors.calibrate()                  #calibrate line sensors 

            flag = 1
            state = "START"                             #set initial fight state
            display.fill(0)
            tuffy = display.load_pbm("zumo_display/signal_received.pbm")
            display.blit(tuffy, 0, 0)
            display.show()
   else:
      #floor_scan()
      proximity_scan()
      #code for proximity
      if state == "START":
         go = random.randrange(1, 2, 1)
         if go == 1:
            motors.set_speeds(6000,6000)
         if go == 2:
            question_mark_kick()
      if state == "DEFAULT":
         print("burst and retreat")
      if state == "RECOVER":
         if right_retreat_flag == 1:
            right_retreat_flag = 0
            right_recovery()
            state = "DEFAULT"
         if left_retreat_flag == 1:
            left_retreat_flag = 0
            left_recovery()
            state = "DEFAULT"
      if state == "RETREAT":
         print("running from opponent robot")
      if state == "CHARGE":
         print("charge when robot centered")

# ===========================================================================================================================================================================================       
# Centralized State-Based Machine Controller 
# Version: 1
#
# Revision History:
# 10/27/25 Jordan Carr, Rohit Sood - Created Version 1 
#
# States: 
#     1. Defense - The Defense state of the machine controller is a specialized, prioritized state designed to immediately recover the robot when one or more of its IR line detectors is
#        above the white line detection threshold. The robot will take different recovery patterns based on the set detection flags. If only one or two of the far-side IR line detectors 
#        is above the threshold, the robot should take a wide sweeping motion away from the edge in the direction of the side with the non-triggered sensors. 
#        If both sides of the 5-sensor array are triggered or the center sensor is fully saturated, the robot should immediately commence a time-based, straight-line reverse. At the 
#        half-way point the robot should make a quick 180 degree turn and continue its trajectory until it reaches the center (i.e. when the timer expires). The 180 degree turn accomplishes two primary goals:
#           a) It allows the robot to scan the arena using its front and side-facing proximity sensors.
#           b) It equally splits time between both blind spot directions. When the robot is initially reversing from the edge it cannot detect anything behind it (i.e. anything further away from the edge). 
#              Turning halfway through the retreat process allows the robot to change its blind spot.
#        The 180 degree turn is a critical opportunity to scan for the opponent and immediately engage in an offensive charge in the event of a detection. 
#     2. Offense
