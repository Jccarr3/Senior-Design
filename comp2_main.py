# ========== IMPORTED FILES ========== #
from zumo_2040_robot import robot
import time
from machine import Pin
import random
from micropython import const
# ==================================== #

# Start Trigger GPIO Pin
trigger = Pin(28, Pin.IN, Pin.PULL_DOWN)

# Start Flag 
flag = 0

# Retreat State Flags
left_retreat_flag , right_retreat_flag = 0, 0

# Controller State Variable
state = "TEST"

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

# IMU 
imu = robot.IMU()
imu.reset()
imu.enable_default()
velocity = 0         #used for calculating speed
acc_index = 0
prev_acc_pull_count = 0
prev_speed_time = 0
INDEX_SIZE = const(50)
acc_vals = [0] * INDEX_SIZE
prev_velocity = 0
contact = 0
# ==================================== #

# ========== MACROS ==========
SENSOR_THRESHOLD = const(1)
MAX_SPEED = const(6000)
TURN_SPEED_MAX = const(1800)
TURN_SPEED_MIN = const(1500)
CHARGE_SPEED = 6000
DECELERATION = const(150)
ACCELERATION = const(150)
DIR_LEFT = const(0)
DIR_RIGHT = const(1)

PRELIMINARY_CHARGE_DURATION_MS = const(300)

ACCL_BUFFER_SIZE = const(10)
DECELERATION_THRESHOLD = const(-80)
ACCELERATION_THRESHOLD = const(80)
# ==================================== #

# ========== TRACKING STATE VARIABLES ==========
sense_dir = DIR_RIGHT
turning_left = False
turning_right = False
turn_speed = TURN_SPEED_MAX
drive_motors = False # Motors are OFF by default
# ==================================== #


# Motors
motors = robot.Motors()



# Testing variablies
prev_time_test = 0
test_count = 0
# ==================================== #


#function for setting up NC STATE identifier
def show_NCSTATE():
   flip = 0
   tuffy = display.load_pbm("zumo_display/tuffy.pbm")
   display.blit(tuffy, 0, 0)

   for led in range(3):
      if (flip) == 0:
         rgbs.set(led,[0,255,0])
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
   motor_control(-1*5800,-1*1800)
   time.sleep_ms(750)
   motor_control(0, 0)
   #right recovery function(used for recovering from edge detection)

   #left recovery function(used for recovering from edge detection)
def left_recovery():
   motor_control(-1*1800,-1*5800)
   time.sleep_ms(750)
   motor_control(0, 0)
   #left recovery function(used for recovering from edge detection)

   #Question mark shape(used as a startup option to get behind opponent)
def question_mark_kick():
   motor_control(-1*1800,-1*5800)
   time.sleep_ms(1100)
   motor_control(-1*5800,-1*1800)
   time.sleep_ms(500)
   motor_control(0, 0)
   #Question mark shape(used as a startup option to get behind opponent

   #Inverse question mark
def inv_question_mark_kick():
   motor_control(-1*5800, -1*1800)
   time.sleep_ms(1100)
   motor_control(-1*1800, -1*5800)
   time.sleep_ms(500)
   motor_control(0, 0)
   #Inverse question mark

   # ========== TRACKING HELPER FUNCTIONS ==========
def turn_right():
    global turning_left, turning_right
    motor_control(turn_speed, -turn_speed)
    turning_left = False
    turning_right = True

def turn_left():
    global turning_left, turning_right
    motor_control(-turn_speed, turn_speed)
    turning_left = True
    turning_right = False

def charge_forward():
    global turning_left, turning_right
    motor_control(CHARGE_SPEED, CHARGE_SPEED)
    turning_left = False
    turning_right = False

def stop():
    global turning_left, turning_right
    motor_control(0, 0)
    turning_left = False
    turning_right = False
   # ==============================   

#ALL FUNCTIONS INVOLVING MOTORS

#Functions involving IMU
   #function to calculate speed of robot
def find_speed():       #calculates speed of robot every 50ms
   global acc_index
   global acc_vals
   global prev_speed_time
   global prev_acc_pull_count
   global velocity
   global prev_velocity

   
   imu.read()
   acceleration = imu.gyro.last_reading_dps

   if acceleration[0] is not None:
      if abs(acceleration[0]) > .2:
         acc_vals[acc_index] = acceleration[0]
      else:
         acc_vals[acc_index] = 0
      
      acc_index += 1

   if acc_index == INDEX_SIZE:
      acc_index = 0
      avg_acc = sum(acc_vals) / INDEX_SIZE * 9.8
      dt = time.ticks_diff(time.ticks_ms(),prev_speed_time) / 1000
      prev_speed_time = time.ticks_ms()
      velocity += avg_acc * dt
      if velocity < 0:
         velocity = 0

# ========== GLOBAL VARIABLES ==========
# Motor Variables
current_left_speed = 0
current_right_speed = 0

# IMU Variables
decel_buffer = [0] * ACCL_BUFFER_SIZE
decel_index = 0
accl_buffer = [0] * ACCL_BUFFER_SIZE
accl_index = 0

# State Machine Variables
detection_duration = 0
attack_exit_condition1 = False
attack_exit_condition2 = False
# ====================================== #

# ========== MOTOR CONTROL FUNCTION ==========
# The function is the main motor API used for controlling the motors. It includes built-in ramp-up functionality to prevent brownout conditions. The
# function uses a blocking periodic ramp-up approach to gradually increase the motor speeds to the target speeds. It incorporates priority calls to 
# the white line detection function, floor_scan(), to ensure that white line detection is not hindered during motor ramp-up. This allows for 
# simplified program flow in othe parts of the code. It is very important that while the motor control function is operating it is always checking for
# white line detection even if its otherwise idle waiting for the next ramp-up point. Since the function is used as the centralized control for the 
# motors, it should maintain and update global speed variables for each motor instead of the program managing the speeds externally. The control 
# function only ramps up the motors, it permits immediate deceleration to the target speeds to allow for quick stops when necessary. The ramp-up 
# should be done in periodic time increments, which is handled in the control function itself via internal timing checks.
def motor_control(target_left_speed, target_right_speed):
   global current_left_speed
   global current_right_speed

   RAMP_UP_INCREMENT = 500      # Speed increment for each ramp-up step
   RAMP_UP_INTERVAL_MS = 50     # Time interval between ramp-up steps

   last_ramp_time = time.ticks_ms() - RAMP_UP_INTERVAL_MS  # Initialize the last ramp-up time variable to allow immediate ramp-up.

   # Clip the target speeds to the maximum allowable speed.
   target_left_speed = max(-MAX_SPEED, min(MAX_SPEED, target_left_speed))
   target_right_speed = max(-MAX_SPEED, min(MAX_SPEED, target_right_speed))

   while (current_left_speed != target_left_speed) or (current_right_speed != target_right_speed):

      # Check if it's time for the next ramp-up step
      if time.ticks_diff(time.ticks_ms(), last_ramp_time) >= RAMP_UP_INTERVAL_MS:
         last_ramp_time = time.ticks_ms()

         # Ramp up left motor speed
         if current_left_speed < target_left_speed:
            current_left_speed = min(current_left_speed + RAMP_UP_INCREMENT, target_left_speed) # Use the min function to avoid overshooting.
         elif current_left_speed > target_left_speed:
            current_left_speed = target_left_speed  # Immediate deceleration

         # Ramp up right motor speed
         if current_right_speed < target_right_speed:
            current_right_speed = min(current_right_speed + RAMP_UP_INCREMENT, target_right_speed) # Use the min function to avoid overshooting.
         elif current_right_speed > target_right_speed:
            current_right_speed = target_right_speed  # Immediate deceleration

         # Set the motor speeds
         motors.set_speeds(current_left_speed, current_right_speed)

      # If the floor scan functino sets the state to RECOVER, exit the motor control function immediately.
      floor_scan()
      if state == "RECOVER":
         break
# ==============================
      
#Functions involving IMU

#main code execution
while True:
   #polling for button input/trigger
   if flag == 0:
      if time.ticks_diff(time.ticks_ms(), prev_time) > 50:
         prev_time = time.ticks_ms()
         if button_a.is_pressed():                       #check for start signal from button A
            line_sensors.calibrate()                     #calibrate line sensors 
            black = line_sensors.read()[2]

            flag = 1
            state = "START"                             #set initial fight state
            tuffy = display.load_pbm("zumo_display/signal_received.pbm")
            display.blit(tuffy, 0, 0)
            display.show()
            time.sleep_ms(1000)
   
   #Main fight code loop
   else:
      floor_scan()
      if time.ticks_diff(time.ticks_ms(), prev_time) > 50:
         prev_time = time.ticks_ms()
         proximity_scan()

      #code for proximity
      if state == "START":
         go = random.randint(1, 3)
         if go == 1:                      #speed blitz
            motor_control(5500,5500)
         if go == 2:                      #question mark shape to get behind opponent 
            question_mark_kick()
         if go == 3:                      #inverse question mark shape to get behind opponent
            inv_question_mark_kick()
         state = "SCAN"

      if state == "SCAN":
         # First, read the proximity sensors to check for the target.
         proximity_sensors.read()
         reading_left = proximity_sensors.left_counts_with_left_leds()
         reading_front_left = proximity_sensors.front_counts_with_left_leds()
         reading_front_right = proximity_sensors.front_counts_with_right_leds()
         reading_right = proximity_sensors.right_counts_with_right_leds()

         # Determine if the target has been seen.
         object_seen = any(reading > SENSOR_THRESHOLD for reading in \
            (reading_left, reading_front_left, reading_front_right, reading_right))
         
         # Adjust the turn speed based on whether an object is seen. If the target is seen, decelerate and begin triangulation for preliminary charge.
         # If the target is not seen, increase scan speed to maintain quick target acquisition.
         if object_seen:
            turn_speed -= DECELERATION
         else:
            turn_speed += ACCELERATION
         turn_speed = min(TURN_SPEED_MAX, max(TURN_SPEED_MIN, turn_speed)) # Constrain the turn speed.

         # If an object is seen, begin triangulation to center the target.
         if object_seen:
            # If the object is centered, execute a preliminary charge.
            if abs(max(reading_left, reading_front_left) - max(reading_right, reading_front_right)) <= 1:
               # Make a call to the motor API to charge forward. The API has built-in ramp-up functionality to prevent a brownout condition.
               # motor_control(CHARGE_SPEED, CHARGE_SPEED) - This is a placeholder for the actual motor API call.
               motor_control(CHARGE_SPEED, CHARGE_SPEED)

               # Option 1: Use a non-blocking timer to allow for limited-duration charge while maintaining white line detection. This was the option used during Competition One.
               # DO NOT USE THE TIMER.SLEEP_MS() FUNCTION FOR NON-CORRECTIVE, LIMITED-DURATION CHARGES. THIS WILL BLOCK THE WHITE LINE DETECTION.
               # Transition to the TIMER state to allow for a non-corrective, non-blocking, limited-duration charge. 
               # back_to = state
               # state = "TIMER"
               # escape_time = time.ticks_ms() + PRELIMINARY_CHARGE_DURATION_MS

               # Option 2: Detection State - Similar to the TIMER state, but monitors the IMU for contact with the opponent during the preliminary charge. This option is designed to enhance
               # opponent lock-on as Option 1 inefficiently exits and re-enters the SCAN state after the limited-duration charge.
               # Transition to a detection state that monitors for contact via the IMU during a preliminary (non-corrective, limited-duration) charge
               state = "DETECTION"
               detection_duration = time.ticks_ms() + PRELIMINARY_CHARGE_DURATION_MS

               pass

            elif max(reading_left, reading_front_left) > max(reading_right, reading_front_right):
               # motor_control(-turn_speed, turn_speed) - This is a placeholder for the actual motor API call.
               motor_control(-turn_speed, turn_speed)
               sense_dir = DIR_LEFT

            elif max(reading_left, reading_front_left) < max(reading_right, reading_front_right):
               # motor_control(turn_speed, -turn_speed) - This is a placeholder for the actual motor API call.
               motor_control(turn_speed, -turn_speed)
               sense_dir = DIR_RIGHT
         else:
            # If an object is not seen, continue turning in the last known direction of the target.
            if sense_dir == DIR_RIGHT:
               # motor_control(turn_speed, -turn_speed) - This is a placeholder for the actual motor API call.
               motor_control(turn_speed, -turn_speed)

               pass

            elif sense_dir == DIR_LEFT:
               # motor_control(-turn_speed, turn_speed) - This is a placeholder for the actual motor API call.
               motor_control(-turn_speed, turn_speed)

               pass
      
      if state == "DETECTION":
         # Monitor the IMU for contact with the opponent during the preliminary charge. This is determined by a sudden deceleration as read by the IMU.

         # Add the current acceleration reading to a buffer and compute the average acceleration when the buffer is full. If the average acceleration exceeds a defined threshold, 
         # transition to the ATTACK state. Note, if the detection duration expires without contact, return to the SCAN state.

         # accl_buffer[accl_index] = IMU.accelerometer
         # accl_index++
         #
         # if (accl_index >= accl_buffer_size) {
         #      avg_accl = avg(accl_buffer)
         #
         #      if avg_accl <= decel_threshold {
         #          state = "ATTACK"  
         #      }
         #     
         #      accl_index = 0
         # }
         # else if (time.ticks_ms() >= detection_duration) {
         #      state = "SCAN"
         # }

         imu.read()
         acceleration = imu.gyro.last_reading_dps
         
         if acceleration[2] is not None:
            decel_buffer[decel_index] = acceleration[2]
            decel_index += 1
         
         if decel_index >= ACCL_BUFFER_SIZE:
            avg_decel = sum(decel_buffer) / ACCL_BUFFER_SIZE

            if avg_decel <= DECELERATION_THRESHOLD:
               state = "ATTACK"

            decel_index = 0
         elif time.ticks_diff(time.ticks_ms(), detection_duration) >= PRELIMINARY_CHARGE_DURATION_MS:
            state = "SCAN"
      
      if state == "ATTACK":
         # This is a high-confidence, high-speed charge towards the opponent. The robot should maintain this state until a white line is detected or the opponent is lost. A controller will
         # be implemented to maintain opponent lock-on via the proximity sensors during the charge. Use the proximity sensor readings to make small adjustments to the motor speeds to maintain 
         # opponent lock-on. 
         # Attack State Exit Options
            # Option 1: Double-Condition Target Loss - If all sensors have lost the opponent AND sudden acceleration is detected via the IMU (indicating the robot has lost contact with the 
            # target), return to the SCAN state. This allows for the controller to take advantage of the recent history of the proximity sensors to try and maintain lock-on. 
            # Option 2: Single-Condition Target Loss - If all sensors have lost the opponent, return to the SCAN state. This is a more aggressive exit condition that may result in faster
            # reacquisition of the opponent, but may also result in premature exit from the ATTACK state.
         # For now we will implement Option 1.
         proximity_sensors.read()
         reading_left = proximity_sensors.left_counts_with_left_leds()
         reading_front_left = proximity_sensors.front_counts_with_left_leds()
         reading_front_right = proximity_sensors.front_counts_with_right_leds()
         reading_right = proximity_sensors.right_counts_with_right_leds()

         object_seen = any(reading > SENSOR_THRESHOLD for reading in \
            (reading_left, reading_front_left, reading_front_right, reading_right))
         
         # Constantly average the acceleration readings to monitor for sudden acceleration indicating loss of contact.
         imu.read()
         acceleration = imu.gyro.last_reading_dps

         if acceleration[2] is not None:
            accl_buffer[accl_index] = acceleration[2]
            accl_index += 1
         
         if accl_index >= ACCL_BUFFER_SIZE:
            avg_accl = sum(accl_buffer) / ACCL_BUFFER_SIZE
            accl_index = 0

            if avg_accl >= ACCELERATION_THRESHOLD:
               # One of the exit conditions has been met, check for the second condition if using Option 1.
               attack_exit_condition1 = True
            else:
               attack_exit_condition1 = False

         
         if object_seen:
            # Reset exit condition flag since the target is still seen.
            attack_exit_condition2 = False

            # Adjust motor speeds based on proximity sensor readings to maintain lock-on.
            left_adjustment = (reading_left + reading_front_left) // 2
            right_adjustment = (reading_right + reading_front_right) // 2

            base_speed = CHARGE_SPEED
            left_speed = base_speed + right_adjustment - left_adjustment # If the left sensors read higher, decrease left motor speed to turn the robot left to realign on the target.
            right_speed = base_speed + left_adjustment - right_adjustment # If the right sensors read higher, decrease right motor speed to turn the robot right to realign on the target.
            
            # Add a robustness check to prevent negative motor speeds.
            left_speed = max(0, left_speed)
            right_speed = max(0, right_speed)

            # Constrain the speeds to maximum allowable speed.
            left_speed = min(left_speed, MAX_SPEED)
            right_speed = min(right_speed, MAX_SPEED)

            motor_control(left_speed, right_speed)
         else:
            # The target is lost, set the exit condition flag.
            attack_exit_condition2 = True

         if attack_exit_condition1 and attack_exit_condition2:
            state = "SCAN"
            attack_exit_condition1 = False
            attack_exit_condition2 = False
      
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
        #  find_speed()
        #  if time.ticks_diff(time.ticks_ms(), prev_time) > 10:
        #     contact += 1
        #     if contact == 5:
        #        prev_velocity = velocity
        #     display.fill(0)
        #     display.text(f"Velocity: {velocity:.2f}",0,0)
        #     display.text(f"Prev: {prev_velocity:.2f}",0,10)
        #     display.show()
        #     if(CHARGE_SPEED < MAX_SPEED):
        #        CHARGE_SPEED += 100
        #        motors.set_speeds(CHARGE_SPEED,CHARGE_SPEED)

         
        #  if(time.ticks_ms() >= escape_time and (velocity > prev_velocity)):
        #     velocity = 0
        #     prev_velocity = 0
        #     CHARGE_SPEED = 2500
        #     state = back_to
        #     rgbs.set(4, [0,0,0])
        #     rgbs.show()

            pass

      if state == "TEST":           #this state is used only for testing new code to be added
         find_speed()
         rgbs.set(4,[0,255,0])
         rgbs.show()
         if time.ticks_diff(time.ticks_ms(), prev_time_test) > 100:
            prev_time_test = time.ticks_ms()
            test_count += 1
            display.fill(0)
            display.text(f"Velocity: {velocity:.2f}",0,0)
            display.show()

         if test_count > 15:
            if time.ticks_diff(time.ticks_ms(), prev_time) > 10:
               if(CHARGE_SPEED < MAX_SPEED):
                  CHARGE_SPEED += 100
                  motors.set_speeds(CHARGE_SPEED,CHARGE_SPEED)

               if velocity < (prev_velocity):
                  rgbs.set(3,[0,255,0])
                  rgbs.show()