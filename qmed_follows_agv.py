#!/usr/bin/env python3

import time
import threading
import logging
from natnet_tracker import NatNetTracker
from coordinate_transform import optitrack_to_ned, quaternion_to_euler
from mavlink_controller import MavlinkController
from safety_config import *
from safety_checks import pre_flight_check, emergency_land, is_altitude_safe

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('optitrack_arm_test.log')
    ]
)
logger = logging.getLogger(__name__)

def main():
    try:
        tracker = NatNetTracker()
        tracker.start()
        mavlink = MavlinkController()
        
        logger.info("Starting position detection phase")
        position_timeout = 30
        
        for i in range(position_timeout):
            status = tracker.get_drone_status()
            
            if tracker.has_drone_position():
                logger.info(f"Position lock SUCCESS! Received {status['position_count']} position updates")
                break
            time.sleep(1)
        
        if not tracker.has_drone_position():
            logger.warning("No OptiTrack position detected - falling back to dummy data")
            use_dummy_position = True
        else:
            use_dummy_position = False
        
        if not use_dummy_position:
            logger.info("Performing pre-flight safety check...")
            if not pre_flight_check(tracker, mavlink):
                logger.error("Pre-flight safety check FAILED - aborting")
                print("Pre-flight safety check FAILED")
                return
            print("Pre-flight safety check PASSED")
        
        running = True
        
        def position_loop():
            while running:
                try:
                    if use_dummy_position:
                        ned_pos = (0.0, 0.0, 0.0)
                        roll, pitch, yaw = 0.0, 0.0, 0.0
                        mavlink.send_vision_position(ned_pos, roll, pitch, yaw)
                    else:
                        pos, quat = tracker.get_drone_position()
                        if pos and quat:
                            ned_pos = optitrack_to_ned(pos)
                            roll, pitch, yaw = quaternion_to_euler(*quat)
                            mavlink.send_vision_position(ned_pos, roll, pitch, yaw)
                except Exception as e:
                    logger.error(f"Position loop error: {e}")
                time.sleep(0.05)  # 20Hz

        def agv_waypoint_loop():
            while running:
                try:
                    if tracker.has_agv_position():
                        agv_pos, agv_orient = tracker.get_agv_position()
                        mavlink.send_agv_relative_waypoint(agv_pos, agv_orient)
                    else:
                        logger.debug("Waiting for AGV position...")
                except Exception as e:
                    logger.error(f"AGV waypoint loop error: {e}")
                time.sleep(1.0)

        threading.Thread(target=position_loop, daemon=True).start()
        time.sleep(3)
        
        mavlink.set_mode('GUIDED')
        time.sleep(2)
        
        mavlink.arm()
        
        time.sleep(3)
        logger.info("Arm command sent, proceeding to take‑off")
        
        safe_altitude = min(MAX_ALTITUDE, 1.0)  
        logger.info(f"Taking off to safe altitude: {safe_altitude}m")
        mavlink.takeoff(safe_altitude)
        threading.Thread(target=agv_waypoint_loop, daemon=True).start()
        
        print(f"System running with ENHANCED safety monitoring - Max altitude: {MAX_ALTITUDE}m")
        logger.info(f"Safety: Max altitude={MAX_ALTITUDE}m, AGV timeout={AGV_LOST_TIMEOUT}s, Battery threshold={BATTERY_CRITICAL}%")
        
        last_position = None
        position_loss_time = None
        
        try:
            while True:
                # SAFETY: Check position loss
                if not use_dummy_position:
                    if not tracker.has_drone_position():
                        if position_loss_time is None:
                            position_loss_time = time.time()
                            logger.warning("Position lost - starting timeout")
                        elif time.time() - position_loss_time > POSITION_TIMEOUT:
                            emergency_land(mavlink, "Position lost")
                            break
                    else:
                        position_loss_time = None

                if tracker.is_agv_lost(AGV_LOST_TIMEOUT):
                    emergency_land(mavlink, f"AGV lost for {AGV_LOST_TIMEOUT}s")
                    break
                
                voltage, percentage = mavlink.get_battery_status()
                if percentage is not None and percentage < BATTERY_CRITICAL:
                    emergency_land(mavlink, f"Critical battery: {percentage}%")
                    break
                
                if voltage is not None and percentage is not None:
                    logger.debug(f"Battery: {voltage:.1f}V ({percentage}%)")
                
                time.sleep(SAFETY_CHECK_INTERVAL)
                
        except KeyboardInterrupt:
            print("\nManual stop requested")
            logger.info("Manual shutdown requested")

        running = False
        tracker.stop()
        print("System stopped")
        
    except Exception as e:
        logger.error(f"Main system error: {e}")
        print(f"System error: {e}")

if __name__ == "__main__":
    main()
