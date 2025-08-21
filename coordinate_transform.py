#!/usr/bin/env python3

import math
import numpy as np

def build_rotation_matrix(rotation_quaternion: np.ndarray):
    q0, q1, q2, q3 = rotation_quaternion.flatten()
    return np.array([
        [2*q0**2 + 2*q1**2 - 1, 2*(q1*q2 - q3*q0),      2*(q1*q3 + q2*q0)],
        [2*(q1*q2 + q3*q0),     2*q0**2 + 2*q2**2 - 1,  2*(q2*q3 - q1*q0)],
        [2*(q1*q3 - q2*q0),     2*(q2*q3 + q1*q0),      2*q0**2 + 2*q3**2 - 1]
    ])

enu2ned_quat = np.array([[0.0], [np.sqrt(2)/2], [np.sqrt(2)/2], [0.0]])
enu2ned_rotmat = build_rotation_matrix(enu2ned_quat)

def optitrack_to_ned(optitrack_pos):
    vec = np.array(optitrack_pos).reshape(3, 1)
    return tuple(np.dot(enu2ned_rotmat, vec).flatten())

def quaternion_to_euler(qx, qy, qz, qw):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def calculate_agv_relative_position(agv_position, agv_orientation, behind_distance=1.0, above_distance=1.0):
    try:
        agv_ned = optitrack_to_ned(agv_position)
        qx, qy, qz, qw = agv_orientation
        _, _, agv_yaw = quaternion_to_euler(qx, qy, qz, qw)
        behind_x = agv_ned[0] - behind_distance * math.cos(agv_yaw)
        behind_y = agv_ned[1] - behind_distance * math.sin(agv_yaw)
        above_z = agv_ned[2] - above_distance  # Negative Z is up in NED
        
        return (behind_x, behind_y, above_z)
    except Exception as e:
        print(f"Error calculating AGV relative position: {e}")
        return (0.0, 0.0, -2.0)
