#!/usr/bin/env python3

import time
import logging
import threading
import signal
import sys
from natnet_tracker import NatNetTracker
from coordinate_transform import optitrack_to_ned, quaternion_to_euler
from mavlink_controller import MavlinkController

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('vision_position_only.log')
    ]
)
logger = logging.getLogger(__name__)

class VisionPositionFeeder:
    def __init__(self):
        self.running = False
        self.tracker = None
        self.mavlink = None
        self.position_thread = None
        
    def start(self):
        try:
            print("=" * 60)
            print("VISION POSITION FEEDER - OptiTrack to MAVLink")
            print("=" * 60)
            print("This script ONLY feeds position data - no flight control")
            print("Use RC transmitter for manual flight control")
            print("=" * 60)
            
            logger.info("Initializing OptiTrack tracker...")
            self.tracker = NatNetTracker()
            self.tracker.start()
            
            logger.info("Initializing MAVLink connection...")
            self.mavlink = MavlinkController()
            
            logger.info("Waiting for OptiTrack position lock...")
            position_timeout = 30
            
            for i in range(position_timeout):
                conn_status = self.tracker.get_connection_status()
                if not conn_status['connected']:
                    print(f"   Connection issue: {conn_status.get('reason', 'Unknown')}")
                    time.sleep(1)
                    continue

                if self.tracker.has_drone_position():
                    status = self.tracker.get_drone_status()
                    logger.info(f"Position lock SUCCESS! Received {status['position_count']} position updates")
                    print(f"   OptiTrack position locked - {status['position_count']} updates received")
                    print(f"   Available rigid bodies: {conn_status['available_bodies']}")
                    print(f"   Server: {conn_status['server_name']}")
                    break
                
                if conn_status['available_bodies']:
                    print(f"   Waiting for drone (ID 5)... Available: {conn_status['available_bodies']} ({i+1}/{position_timeout})")
                else:
                    print(f"   Waiting for any rigid bodies... ({i+1}/{position_timeout})")
                time.sleep(1)
            else:
                conn_status = self.tracker.get_connection_status()
                print(" No OptiTrack position detected for drone (rigid body ID 5)")
                print(f"   Connection status: {conn_status['connected']}")
                print(f"   Available rigid bodies: {conn_status['available_bodies']}")
                print(f"   Total updates received: {conn_status['total_updates']}")
                return False
            
            self.running = True
            self.position_thread = threading.Thread(target=self._position_loop, daemon=True)
            self.position_thread.start()
            
            print("\n Vision position feeding ACTIVE")
            print("   - Sending VISION_POSITION_ESTIMATE at 20Hz")
            print("   - Flight controller should show EKF position updates")
            print("   - Use RC transmitter for manual flight control")
            print("   - Press Ctrl+C to stop")
            print()
            
            self._status_loop()
            
            return True

        except Exception as e:
            logger.error(f"Initialization failed: {e}")
            print(f"Initialization failed: {e}")
            return False

    def _position_loop(self):
        position_count = 0
        last_log_time = time.time()
        last_warning_time = 0
        
        logger.info("Position feeding loop started at 20Hz")
        
        while self.running:
            try:
                pos, quat = self.tracker.get_drone_position()

                if self.tracker.is_receiving_data() and pos and quat:
                    ned_pos = optitrack_to_ned(pos)
                    roll, pitch, yaw = quaternion_to_euler(*quat)

                    self.mavlink.send_vision_position(ned_pos, roll, pitch, yaw)
                    position_count += 1
                        
                    if time.time() - last_log_time > 10.0:
                        logger.info(f"Position updates sent: {position_count} | Current NED: {ned_pos[0]:.2f}, {ned_pos[1]:.2f}, {ned_pos[2]:.2f}")
                        last_log_time = time.time()

                else:
                    current_time = time.time()
                    if current_time - last_warning_time > 5.0:
                        logger.warning("No position data available")
                        last_warning_time = current_time

            except Exception as e:
                logger.error(f"Position loop error: {e}")
                
            time.sleep(0.05)

        logger.info(f"Position feeding loop stopped. Total updates sent: {position_count}")

    def _status_loop(self):
        last_status_time = time.time()
        status_interval = 5.0

        try:
            while self.running:
                current_time = time.time()
                if current_time - last_status_time >= status_interval:
                    if not self.tracker.is_receiving_data():
                        print("No data received recently - check OptiTrack connection")
                        conn_status = self.tracker.get_connection_status()
                        print(f"   Connection: {conn_status['connected']}, Bodies: {conn_status['available_bodies']}")

                    elif not self.tracker.has_drone_position():
                        print("Drone position lost - check rigid body ID 5")
                        conn_status = self.tracker.get_connection_status()
                        if conn_status['available_bodies']:
                            print(f"   Available bodies: {conn_status['available_bodies']} (waiting for ID 5)")

                    else:
                        status = self.tracker.get_drone_status()
                        if status['has_position']:
                            pos = status['current_position']
                            ned_pos = optitrack_to_ned(pos)
                            print(f"Drone ID 5: NED({ned_pos[0]:.2f}, {ned_pos[1]:.2f}, {ned_pos[2]:.2f}) | Updates: {status['position_count']}")
                    
                    last_status_time = current_time

                time.sleep(1.0)

        except KeyboardInterrupt:
            print("\nShutdown requested...")
            self.stop()

    def stop(self):
        logger.info("Stopping vision position feeder...")
        self.running = False
        
        if self.tracker:
            self.tracker.stop()

        print("Vision position feeding stopped")
        print("Flight controller will fall back to other position sources")

def signal_handler(sig, frame):
    print("\nInterrupt received...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    feeder = VisionPositionFeeder()

    try:
        success = feeder.start()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        feeder.stop()
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        print(f"Unexpected error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
