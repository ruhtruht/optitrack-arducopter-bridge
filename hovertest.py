#!/usr/bin/env python3

import time
import logging
import threading
from natnet_tracker import NatNetTracker
from coordinate_transform import optitrack_to_ned, quaternion_to_euler
from mavlink_controller import MavlinkController
from safety_config import MAX_ALTITUDE, HOVER_TEST_DURATION
from safety_checks import pre_flight_check, emergency_land

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('hovertest.log')
    ]
)
logger = logging.getLogger(__name__)

def hover_test():
    print("=" * 60)
    print("HOVER TEST - Minimalistic Safety Test")
    print("=" * 60)
    
    try:
        logger.info("Initializing system...")
        tracker = NatNetTracker()
        tracker.start()
        mavlink = MavlinkController()

        logger.info("Waiting for OptiTrack position...")
        for i in range(15):
            if tracker.has_drone_position():
                logger.info("OptiTrack position received")
                break
            time.sleep(1)
        else:
            print("No OptiTrack position - test aborted")
            return False

        print("\nPre-flight Safety Check...")
        if not pre_flight_check(tracker, mavlink):
            print("Pre-flight Check FAILED - test aborted")
            return False
        print("Pre-flight Check PASSED")

        running = True
        def position_loop():
            while running:
                try:
                    pos, quat = tracker.get_drone_position()
                    if pos and quat:
                        ned_pos = optitrack_to_ned(pos)
                        roll, pitch, yaw = quaternion_to_euler(*quat)
                        mavlink.send_vision_position(ned_pos, roll, pitch, yaw)
                except Exception as e:
                    logger.error(f"Position loop error: {e}")
                time.sleep(0.05)

        threading.Thread(target=position_loop, daemon=True).start()
        time.sleep(2)

        print("\nSetting GUIDED mode and arming drone...")
        mavlink.set_mode('GUIDED')
        time.sleep(2)
        mavlink.arm()

        for i in range(10):
            if mavlink.is_armed():
                print("Drone is ARMED")
                break
            time.sleep(1)
        else:
            print("Arming failed - test aborted")
            return False

        hover_altitude = min(0.8, MAX_ALTITUDE)
        print(f"\nTakeoff to {hover_altitude}m altitude...")
        mavlink.takeoff(hover_altitude)

        time.sleep(8)

        print(f"\nhover test - {HOVER_TEST_DURATION} seconds at {hover_altitude}m")
        print("   Monitoring: Position, battery, stability")
        print("   Press Ctrl+C for emergency landing")
        
        start_time = time.time()
        try:
            while time.time() - start_time < HOVER_TEST_DURATION:
                if not tracker.has_drone_position():
                    emergency_land(mavlink, "Position lost")
                    break

                voltage, percentage = mavlink.get_battery_status()
                if percentage and percentage < 20:
                    emergency_land(mavlink, f"Battery critical: {percentage}%")
                    break

                remaining = HOVER_TEST_DURATION - (time.time() - start_time)
                if int(remaining) % 5 == 0 and remaining > 0:
                    print(f"   {int(remaining)}s remaining - Battery: {percentage}%" if percentage else f"   {int(remaining)}s remaining")

                time.sleep(1)
            
            print("hover test completed successfully!")

        except KeyboardInterrupt:
            print("\nManual stop - emergency landing initiated")
            emergency_land(mavlink, "Manual stop")

        print("\nAutomatic landing...")
        mavlink.land()
        time.sleep(5)

        running = False
        tracker.stop()
        
        print("\n" + "=" * 60)
        print("hover TEST COMPLETED")
        print("=" * 60)
        return True

    except Exception as e:
        logger.error(f"hover test error: {e}")
        print(f"System error: {e}")
        return False

if __name__ == "__main__":
    success = hover_test()
    exit(0 if success else 1)
