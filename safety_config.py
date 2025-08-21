#!/usr/bin/env python3

# FLIGHT SAFETY LIMITS
MAX_ALTITUDE = 1.0          # meters (conservative, well below ceiling)
MIN_ALTITUDE = 0.2          # meters (minimum safe altitude)
MAX_VELOCITY = 0.5          # m/s (conservative speed limit)
MAX_ACCELERATION = 1.0      # m/s² (gentle acceleration)

# POSITION VALIDATION
POSITION_TIMEOUT = 0.5      # seconds (max time without position update)
MAX_POSITION_JUMP = 0.5     # meters (max position change per update)
POSITION_ACCURACY_THRESHOLD = 0.05  # meters (required accuracy)
MIN_TRACKING_CONFIDENCE = 0.8       # minimum tracking confidence

# EMERGENCY THRESHOLDS
BATTERY_CRITICAL = 15       # percentage (emergency landing)
BATTERY_WARNING = 25        # percentage (warning level)
AGV_LOST_TIMEOUT = 10.0     # seconds (reduced from 15s)
COMMUNICATION_TIMEOUT = 2.0 # seconds (MAVLink communication)

# PRE-FLIGHT VALIDATION
MIN_POSITION_SAMPLES = 50   # minimum position samples before flight
POSITION_STABILITY_TIME = 5.0  # seconds of stable position required
SYSTEM_CHECK_TIMEOUT = 30.0    # seconds for system initialization

# FLIGHT PHASES
PHASE_GROUND_TEST = "ground_test"
PHASE_HOVER_TEST = "hover_test"
PHASE_MOVEMENT_TEST = "movement_test"
PHASE_AGV_FOLLOW = "agv_follow"

# SAFETY MONITORING
SAFETY_CHECK_INTERVAL = 0.5  # seconds (increased frequency)
STATUS_REPORT_INTERVAL = 2.0 # seconds
LOG_LEVEL_FLIGHT = "INFO"    # logging level during flight

# EMERGENCY ACTIONS
EMERGENCY_LAND_ALTITUDE = 0.3  # meters (emergency landing target)
EMERGENCY_DESCENT_RATE = 0.2   # m/s (controlled emergency descent)

# TEST PROTOCOL SETTINGS
HOVER_TEST_DURATION = 10.0     # seconds
MOVEMENT_TEST_DISTANCE = 0.5   # meters (small test movements)
AGV_FOLLOW_MAX_DISTANCE = 2.0  # meters (max distance from AGV)

class SafetyLimits:
    @staticmethod
    def is_altitude_safe(altitude):
        return MIN_ALTITUDE <= abs(altitude) <= MAX_ALTITUDE
    
    @staticmethod
    def is_velocity_safe(velocity):
        if isinstance(velocity, (list, tuple)):
            return all(abs(v) <= MAX_VELOCITY for v in velocity)
        return abs(velocity) <= MAX_VELOCITY
    
    @staticmethod
    def is_position_jump_safe(old_pos, new_pos):
        if old_pos is None or new_pos is None:
            return True
        
        distance = ((new_pos[0] - old_pos[0])**2 + 
                   (new_pos[1] - old_pos[1])**2 + 
                   (new_pos[2] - old_pos[2])**2)**0.5
        
        return distance <= MAX_POSITION_JUMP
