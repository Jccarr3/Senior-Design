from zumo_2040_robot import robot
import time
from machine import Pin
import random

led = Pin(25, Pin.OUT)
trigger = Pin(28, Pin.IN, Pin.PULL_DOWN)
flag = 0
left_retreat_flag , right_retreat_flag = 0, 0 
state = "START"
prev_time = 0

display = robot.Display()

rgbs = robot.RGBLEDs()
rgbs.set_brightness(3)

button_a = robot.ButtonA()

button_b = robot.ButtonB()

line_sensors = robot.LineSensors()

proximity_sensors = robot.ProximitySensors()

motors = robot.Motors()

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

show_NCSTATE()

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
         
#def proximity_scan():
#   proximity_sensors.read()

#   readings = [proximity_sensors.left_counts_with_left_leds(),]

def right_recovery():
   motors.set_speeds(-1*5800,-1*1800)
   time.sleep_ms(750)
   motors.off()

def left_recovery():
   motors.set_speeds(-1*1800,-1*5800)
   time.sleep_ms(750)
   motors.off()

def question_mark_kick():
   motors.set_speeds(-1*1800,-1*5800)
   time.sleep_ms(1100)
   motors.set_speeds(-1*5800,-1*1800)
   time.sleep_ms(300)
   motors.off()

# ========== TRACKING CONSTANTS ==========
SENSOR_THRESHOLD = const(1)
TURN_SPEED_MAX = const(6000)
TURN_SPEED_MIN = const(1500)
CHARGE_SPEED = const(1500)
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

while True:
   if flag == 0:
      if time.ticks_diff(time.ticks_ms(), prev_time) > 50:
         prev_time = time.ticks_ms()
         if button_a.is_pressed():
        
            line_sensors.calibrate()                 
            
            flag = 1
            state = "TRACK"
            
            display.fill(0)
            tuffy = display.load_pbm("zumo_display/signal_received.pbm")
            display.blit(tuffy, 0, 0)
            display.show()
   else:  
      if state == "START":
         go = random.randrange(1, 3)
         if go == 1:
            motors.set_speeds(6000,6000)
         if go == 2:
            question_mark_kick()
         state = "DEFAULT"

      elif state == "DEFAULT":
         print("burst and retreat")
         pass

      elif state == "RECOVER":
         if right_retreat_flag == 1:
            right_retreat_flag = 0
            right_recovery()
            state = "DEFAULT"
         if left_retreat_flag == 1:
            left_retreat_flag = 0
            left_recovery()
            state = "DEFAULT"

      elif state == "RETREAT":
         print("running from opponent robot")
         pass

      elif state == "CHARGE":
         print("charge when robot centered")
         pass
         
      # ========== TRACKING ==========   
      elif state == "TRACK":
         proximity_sensors.read()
         reading_left = proximity_sensors.left_counts_with_left_leds()
         reading_front_left = proximity_sensors.front_counts_with_left_leds()
         reading_front_right = proximity_sensors.front_counts_with_right_leds()
         reading_right = proximity_sensors.right_counts_with_right_leds()

         # If the user presses button B, toggle whether the motors are on.
         if button_b.check() == True:
            while button_b.check() != False: pass  # wait for release
            drive_motors = not drive_motors
            if drive_motors:
                time.sleep_ms(250) # Give user time to let go
            else:
                stop()

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
            if max(reading_left, reading_front_left) < max(reading_right, reading_front_right):
               # Object is to the right, turn right
               if drive_motors: turn_right()
               sense_dir = DIR_RIGHT

            elif max(reading_left, reading_front_left) > max(reading_right, reading_front_right):
               # Object is to the left, turn left
               if drive_motors: turn_left()
               sense_dir = DIR_LEFT

            elif max(reading_left, reading_front_left) == max(reading_right, reading_front_right):
               # Object is centered, charge forward
                if drive_motors:
                     charge_forward()  # This will now run
         else:
            # --- Object is NOT Seen ---
            # Keep turning in the direction we last sensed the object.
            if sense_dir == DIR_RIGHT:
               if drive_motors: turn_right() # This will now run

            elif sense_dir == DIR_LEFT:
               if drive_motors: turn_left() # This will now run
      # ==============================