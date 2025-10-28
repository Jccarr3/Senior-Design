# ========== IMPORTED FILES ========== #
from zumo_2040_robot import robot
import time
from machine import Pin
import random
# ==================================== #

# Start Trigger GPIO Pin
trigger = Pin(28, Pin.IN, Pin.PULL_DOWN)

# Start Flag 
flag = 0

# Retreat State Flags
left_retreat_flag , right_retreat_flag = 0, 0

# Controller State Variable
state = "START"

# Period Control
prev_time = 0 

# ========== Robot Objects ========== #
# Display
display = robot.Display()

# LEDs
rgbs = robot.RGBLEDs()
rgbs.set_brightness(1)

# Buttons
button_a = robot.ButtonA()

# Line Sensors
line_sensors = robot.LineSensors()
black = 0

# Proximity Sensors
proximity_sensors = robot.ProximitySensors()
   # ========== TRACKING CONSTANTS ==========
SENSOR_THRESHOLD = const(1)
MAX_SPEED = const(6000)
TURN_SPEED_MAX = const(1800)
TURN_SPEED_MIN = const(1500)
CHARGE_SPEED = 2500
DECELERATION = const(150)
ACCELERATION = const(150)
DIR_LEFT = const(0)
DIR_RIGHT = const(1)

   # ========== TRACKING STATE VARIABLES ==========
sense_dir = DIR_RIGHT
turning_left = False
turning_right = False
turn_speed = TURN_SPEED_MAX
drive_motors = False # Motors are OFF by default

# Motors
motors = robot.Motors()
# ==================================== #


#function for setting up NC STATE identifier
def show_NCSTATE():
   flip = 0
   tuffy = display.load_pbm("zumo_display/tuffy.pbm")
   display.blit(tuffy, 0, 0)

   for led in range(3):
      if (flip) == 0:
         rgbs.set(led,[255,0,0])
      else:
         rgbs.set(led,[200,200,200])

      if(led == 2): flip = not flip
      flip = not flip

   rgbs.show()
   display.show()
#function for setting up NC STATE identifier


#setup team identifier
show_NCSTATE()
#setup team identifier

#line detection function
def floor_scan():
   global left_retreat_flag, right_retreat_flag, state
   line_sensors.start_read(emitters_on = True)
   time.sleep_ms(2)

   line = line_sensors.read()
   for i in range(5):
      if((line[i] < (black - 300)) and (i <= 1)):
         state = "RECOVER"
         right_retreat_flag = 1
         break
      elif((line[i] < (black - 300)) and (i > 2)):
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
   time.sleep_ms(500)
   motors.off()
   #Question mark shape(used as a startup option to get behind opponent

   # ========== TRACKING HELPER FUNCTIONS ==========
def turn_right():
    global turning_left, turning_right
    motors.set_speeds(turn_speed, -turn_speed)
    turning_left = False
    turning_right = True

def turn_left():
    global turning_left, turning_right
    motors.set_speeds(-turn_speed, turn_speed)
    turning_left = True
    turning_right = False

def charge_forward():
    global turning_left, turning_right
    motors.set_speeds(CHARGE_SPEED, CHARGE_SPEED)
    turning_left = False
    turning_right = False

def stop():
    global turning_left, turning_right
    motors.set_speeds(0, 0)
    turning_left = False
    turning_right = False
   # ==============================   

#ALL FUNCTIONS INVOLVING MOTORS

#main code execution
while True:
   #polling for button input/trigger
   if flag == 0:
      if time.ticks_diff(time.ticks_ms(), prev_time) > 50:
         prev_time = time.ticks_ms()
         if button_a.is_pressed():                       #check for start signal from button A
            line_sensors.calibrate()                  #calibrate line sensors 
            black = line_sensors.read()[2]

            flag = 1
            state = "START"                             #set initial fight state
            tuffy = display.load_pbm("zumo_display/signal_received.pbm")
            display.blit(tuffy, 0, 0)
            display.show()
            #time.sleep_ms(1000)
   
   #Main fight code loop
   else:
      floor_scan()

      if time.ticks_diff(time.ticks_ms(), prev_time) > 50:
         prev_time = time.ticks_ms()
         proximity_scan()
      #code for proximity
      if state == "START":
         go = random.randint(1, 2)
         if go == 1:                      #speed blitz
            motors.set_speeds(5500,5500)
         if go == 2:                      #question mark shape to get behind opponent 
            question_mark_kick()
         state = "ATTACK"
      if state == "ATTACK":
         proximity_sensors.read()
         reading_left = proximity_sensors.left_counts_with_left_leds()
         reading_front_left = proximity_sensors.front_counts_with_left_leds()
         reading_front_right = proximity_sensors.front_counts_with_right_leds()
         reading_right = proximity_sensors.right_counts_with_right_leds()



         # Determine if an object is visible or not.
         object_seen = any(reading > SENSOR_THRESHOLD for reading in \
            (reading_left, reading_front_left, reading_front_right, reading_right))

         if object_seen:
            # An object is visible, so we will start decelerating
            turn_speed -= DECELERATION
         else:
            # An object is not visible, so we will accelerate
            turn_speed += ACCELERATION

         # Constrain the turn speed
         turn_speed = min(TURN_SPEED_MAX, max(TURN_SPEED_MIN, turn_speed))
         
         if object_seen:
            # --- Object IS Seen ---
            if abs(max(reading_left, reading_front_left) - max(reading_right, reading_front_right)) <= 1:
               # Object is centered, charge forward
               back_to = state
               state = "TIMER"
               escape_time = time.ticks_ms() + 300
               charge_forward()  # This will now run
               rgbs.set(4, [255,255,255])
               rgbs.show()

            elif max(reading_left, reading_front_left) > max(reading_right, reading_front_right):
               # Object is to the left, turn left
               turn_left()
               sense_dir = DIR_LEFT

            elif max(reading_left, reading_front_left) < max(reading_right, reading_front_right):
                # Object is to the right, turn right
               turn_right()
               sense_dir = DIR_RIGHT
               
         else:
            # --- Object is NOT Seen ---
            # Keep turning in the direction we last sensed the object.
            if sense_dir == DIR_RIGHT:
               turn_right() # This will now run

            elif sense_dir == DIR_LEFT:
               turn_left() # This will now run
      # ==============================
      if state == "RECOVER":
         rgbs.set(4, [0,0,0])
         rgbs.show()
         if right_retreat_flag == 1:
            right_retreat_flag = 0
            right_recovery()
            state = "ATTACK"
         if left_retreat_flag == 1:
            left_retreat_flag = 0
            left_recovery()
            state = "ATTACK"

      if state == "TIMER":
         if time.ticks_diff(time.ticks_ms(), prev_time) > 10:
            if(CHARGE_SPEED < MAX_SPEED):
               CHARGE_SPEED += 85
               motors.set_speeds(CHARGE_SPEED,CHARGE_SPEED)
         if(time.ticks_ms() >= escape_time):
            CHARGE_SPEED = 2500
            state = back_to
            rgbs.set(4, [0,0,0])
            rgbs.show()
            
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
#        The 180 degree turn is a critical opportunity to scan for the opponent and immediately engage in an offensive charge if the turn revealed the targets direction and/or position. 
#     2. Offense
