#!/usr/bin/env python3

import time
import logging
from safety_config import *

logger = logging.getLogger(__name__)

def pre_flight_check(tracker, mavlink) -> bool:
    logger.info("=== PRE-FLIGHT SAFETY CHECK ===")
    logger.info("Checking OptiTrack stability...")
    stable_count = 0
    for _ in range(50): 
        if tracker.has_drone_position():
            stable_count += 1
        time.sleep(0.1)
    
    if stable_count < 40:  
        logger.error(f"OptiTrack unstable: {stable_count}/50 samples")
        return False
    
    logger.info("Checking battery...")
    voltage, percentage = mavlink.get_battery_status()
    if percentage and percentage < BATTERY_WARNING:
        logger.error(f"Battery too low: {percentage}%")
        return False
    logger.info("✓ Pre-flight check PASSED")
    return True

def is_altitude_safe(altitude) -> bool:
    return abs(altitude) <= MAX_ALTITUDE

def emergency_land(mavlink, reason="Emergency"):
    logger.critical(f"EMERGENCY LANDING: {reason}")
    print(f" EMERGENCY LANDING: {reason}")
    mavlink.land()

def validate_position_change(old_pos, new_pos) -> bool:
    if not old_pos or not new_pos:
        return True
    
    distance = ((new_pos[0] - old_pos[0])**2 + 
               (new_pos[1] - old_pos[1])**2 + 
               (new_pos[2] - old_pos[2])**2)**0.5
    
    return distance <= MAX_POSITION_JUMP
