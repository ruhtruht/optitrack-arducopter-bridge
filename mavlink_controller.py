#!/usr/bin/env python3

from pymavlink import mavutil
import time
import logging
import threading
from coordinate_transform import calculate_agv_relative_position
from safety_config import MAX_ALTITUDE

logger = logging.getLogger(__name__)

class MavlinkController:
    def __init__(self, connection_string="/dev/ttyUSB1", baud=921600):
        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=baud)
            self.master.wait_heartbeat()
            self._lock = threading.Lock()  # For thread-safe operations
            logger.info(f"MAVLink connected: {connection_string} @ {baud}")
        except Exception as e:
            logger.error(f"Failed to connect to MAVLink: {e}")
            raise

    def send_vision_position(self, ned_pos, roll, pitch, yaw):
        """Send VISION_POSITION_ESTIMATE to ArduPilot"""
        timestamp_us = int(time.time() * 1e6)
        
        with self._lock:
            self.master.mav.vision_position_estimate_send(
                timestamp_us,  # us Timestamp (UNIX time or time since system boot)
                ned_pos[0],    # x Global X position
                ned_pos[1],    # y Global Y position  
                ned_pos[2],    # z Global Z position
                roll,          # roll Roll angle
                pitch,         # pitch Pitch angle
                yaw            # yaw Yaw angle
            )

    def send_agv_relative_waypoint(self, agv_position, agv_orientation):
        """Send waypoint 1m above and 1m behind AGV position"""
        if agv_position is None or agv_orientation is None:
            logger.warning("No AGV position/orientation available")
            return False

        # Calculate waypoint position using coordinate transform function
        waypoint_behind_above = calculate_agv_relative_position(agv_position, agv_orientation)
        
        # Send waypoint command
        with self._lock:
            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                0,  # seq
                3,  # frame (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                16, # command (MAV_CMD_NAV_WAYPOINT)
                2,  # current (2 = guided mode waypoint)
                1,  # autocontinue
                0,  # param1 (hold time)
                0,  # param2 (acceptance radius)
                0,  # param3 (pass radius)
                0,  # param4 (yaw)
                waypoint_behind_above[0],  # x (latitude or local x)
                waypoint_behind_above[1],  # y (longitude or local y)
                waypoint_behind_above[2]   # z (altitude or local z)
            )
        
        logger.info(f"Sent waypoint relative to AGV: {waypoint_behind_above}")
        return True

    def set_mode(self, mode):
        mode_mapping = {
            'MANUAL': 0,
            'STABILIZE': 0,
            'GUIDED': 4,
            'AUTO': 3,
            'LAND': 9
        }
        
        if mode in mode_mapping:
            with self._lock:
                self.master.set_mode(mode_mapping[mode])

    def arm(self):
        with self._lock:
            self.master.arducopter_arm()

    def takeoff(self, altitude):
        # SAFETY: Limit takeoff altitude
        safe_altitude = min(altitude, MAX_ALTITUDE)
        if safe_altitude != altitude:
            logger.warning(f"Takeoff altitude limited: {altitude}m -> {safe_altitude}m")
        
        logger.info(f"Takeoff command: {safe_altitude}m altitude")
        with self._lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, safe_altitude
            )

    def land(self):
        with self._lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
        logger.info("Landing command sent")

    def get_battery_status(self):
        try:
            with self._lock:
                # Request battery status
                msg = self.master.recv_match(type='BATTERY_STATUS', blocking=False, timeout=0.1)
                if msg:
                    voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0
                    percentage = msg.battery_remaining if msg.battery_remaining != -1 else 0
                    return voltage, percentage
                return None, None
        except Exception as e:
            logger.debug(f"Battery status error: {e}")
            return None, None

    def is_armed(self):
        with self._lock:
            return self.master.motors_armed()

    def get_mode(self):
        with self._lock:
            return self.master.flightmode
