# This file defines a class for tracking an opponent.
# Save this file as "opponent_tracker.py" in the same directory
# as your main competition program.

# We import "const" from micropython for a performance optimization.
# This is like using "const int" in C.
from micropython import const

# We don't need to import the full "robot" object here,
# because we will pass the hardware objects (motors, sensors)
# into the constructor. This is a C++/OOP best practice
# called "Dependency Injection."

class OpponentTracker:
    """
    A class to handle opponent tracking.
    Create one instance of this class in your main program
    and call its update() method once per loop.
    """

    # --- Constants ---
    # These are defined as class variables.
    # You can tune these values here.
    SENSOR_THRESHOLD = const(2)        # Minimum total sensor reading to "see" an object.
    TURN_SPEED_MAX = const(5000)       # Max scan/turn speed (6000 is full).
    TURN_SPEED_MIN = const(1500)       # Min scan/turn speed.
    DECELERATION = const(150)          # How fast to slow down when an object is seen.
    ACCELERATION = const(150)          # How fast to speed up when scanning.
    ANGLE_DEADZONE = const(5)          # How many degrees off-center is "good enough" (to stop jitter).

    # --- Directions (for scanning) ---
    _DIR_LEFT = const(0)
    _DIR_RIGHT = const(1)

    # --- RGB LED Colors ---
    # Using tuples (immutable lists) to store (R, G, B) values.
    _RGB_OFF = (0, 0, 0)
    _RGB_GREEN = (0, 255, 0)     # Locked on
    _RGB_YELLOW = (255, 64, 0)   # Turning
    _RGB_RED = (255, 0, 0)       # Scanning

    def __init__(self, motors, proximity_sensors, rgb_leds):
        """
        Constructor for the OpponentTracker.
        This is the C++ equivalent of:
        OpponentTracker::OpponentTracker(Motors* m, ProximitySensors* p, ...)
        """

        # --- Store Hardware References ---
        # We store the objects passed in as member variables ("self.").
        # This is like "this->motors = m;" in C++.
        self.motors = motors
        self.proximity_sensors = proximity_sensors
        self.rgb_leds = rgb_leds

        # --- Internal State Variables ---
        # These are the member variables that track our state.
        
        # Last known direction of the opponent. Default to right.
        self.sense_dir = self._DIR_RIGHT
        
        # Current speed for turning. Start at max.
        self.turn_speed = self.TURN_SPEED_MAX
        
        # This is the "on/off" switch for the tracking behavior.
        # Your main program can control this.
        self.enabled = False

    def set_enabled(self, enabled):
        """
        Public method to turn the tracking behavior on or off.
        Call this from your main program (e.g., when a button is pressed).
        """
        self.enabled = enabled
        
        # If we are disabling, make sure to stop the motors.
        if not self.enabled:
            self.motors.set_speeds(0, 0)
            self._set_leds(self._RGB_OFF, self._RGB_OFF, self._RGB_OFF)

    def _set_leds(self, left, front, right):
        """
        Internal "private" helper method to set the front LEDs.
        The underscore convention "_" means "private."
        """
        self.rgb_leds.set(5, left)
        self.rgb_leds.set(4, front)
        self.rgb_leds.set(3, right)
        self.rgb_leds.show()

    def update(self):
        """
        This is the main "tick" function.
        Call this ONCE per loop from your main "while True:" loop.
        """
        
        # --- Step 1: Read the sensors ---
        # This tells the sensor object to read all 6 values
        # and store them internally.
        self.proximity_sensors.read()

        # --- Step 2: Check for an object ---
        # We use total_counts() which sums all 6 readings.
        # This is more robust than just checking a few.
        total = self.proximity_sensors.total_counts()
        object_seen = total > self.SENSOR_THRESHOLD

        # --- Step 3: Adjust Speed ---
        # This logic is identical to the example program.
        if object_seen:
            # Object is visible, slow down to "lock on"
            self.turn_speed -= self.DECELERATION
        else:
            # No object, speed up to scan faster
            self.turn_speed += self.ACCELERATION

        # Constrain the turn speed between min and max.
        # Python's min() and max() are great for this.
        self.turn_speed = min(self.TURN_SPEED_MAX, max(self.TURN_SPEED_MIN, self.turn_speed))

        # --- Step 4: Check if we are even enabled ---
        # If the user has called set_enabled(False), we don't
        # want to do any motor control.
        if not self.enabled:
            return  # Exit the function early.

        # --- Step 5: Main Decision Logic ---
        if object_seen:
            # --- Object is SEEN ---
            # Get the angle estimate (-90 to +90) from the sensor object.
            # This "uses everything" about the sensors.
            angle = self.proximity_sensors.angle_estimate()

            if angle < -self.ANGLE_DEADZONE:
                # Object is to the left. Turn left.
                self.motors.set_speeds(-self.turn_speed, self.turn_speed)
                self.sense_dir = self._DIR_LEFT
                self._set_leds(self._RGB_YELLOW, self._RGB_OFF, self._RGB_OFF)
            
            elif angle > self.ANGLE_DEADZONE:
                # Object is to the right. Turn right.
                self.motors.set_speeds(self.turn_speed, -self.turn_speed)
                self.sense_dir = self._DIR_RIGHT
                self._set_leds(self._RGB_OFF, self._RGB_OFF, self._RGB_YELLOW)
            
            else:
                # Object is centered (within the deadzone). Stop.
                self.motors.set_speeds(0, 0)
                self._set_leds(self._RGB_OFF, self._RGB_GREEN, self._RGB_OFF)
        
        else:
            # --- Object is NOT SEEN ---
            # Keep scanning in the last known direction.
            if self.sense_dir == self._DIR_RIGHT:
                self.motors.set_speeds(self.turn_speed, -self.turn_speed)
                self._set_leds(self._RGB_OFF, self._RGB_OFF, self._RGB_RED)
            else:
                self.motors.set_speeds(-self.turn_speed, self.turn_speed)
                self._set_leds(self._RGB_RED, self._RGB_OFF, self._RGB_OFF)