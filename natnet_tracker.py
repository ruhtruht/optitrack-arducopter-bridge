#!/usr/bin/env python3
import sys
import os
sys.path.append('../natnet_followagv')

from NatNetClient import NatNetClient
import time
import logging
import threading

logger = logging.getLogger(__name__)

class NatNetTracker:
    def __init__(self, server_ip="192.168.41.119", client_ip="192.168.41.124"):
        self.server_ip = server_ip
        self.client_ip = client_ip
        self.client = None
        self.connected = False
        self.connection_timeout = 10.0
        
        self.bodies = {}
        
        self._lock = threading.Lock()
        self._last_rx_any = None
        
    def start(self):
        try:
            logger.info(f"Initializing NatNet client: {self.client_ip} -> {self.server_ip}")
            
            self.client = NatNetClient()
            self.client.set_client_address(self.client_ip)
            self.client.set_server_address(self.server_ip)
            self.client.set_use_multicast(False)
            
            self.client.rigid_body_listener = self._on_rigid_body
            
            self.client.set_print_level(0)
            
            logger.info("Starting NatNet client threads...")
            success = self.client.run(thread_option="d")  
            
            if not success:
                raise Exception("Failed to start NatNet client threads")
            
            logger.info("Waiting for NatNet connection establishment...")
            connection_start = time.time()
            
            while not self.client.connected() and (time.time() - connection_start) < self.connection_timeout:
                time.sleep(0.1)
            
            if not self.client.connected():
                raise Exception(f"Failed to establish NatNet connection within {self.connection_timeout}s")
            
            logger.info("Requesting model definitions...")
            self.client.send_request(self.client.command_socket, 
                                   self.client.NAT_REQUEST_MODELDEF, 
                                   "", 
                                   (self.server_ip, self.client.command_port))
            time.sleep(1.0)
            self.connected = True
            logger.info(f"NatNet client successfully connected and streaming")
            logger.info(f"Server: {self.client.get_application_name()}")
            logger.info(f"NatNet Version: {'.'.join(map(str, self.client.get_nat_net_version_server()))}")
            
        except Exception as e:
            logger.error(f"Failed to start NatNet client: {e}")
            if self.client:
                try:
                    self.client.shutdown()
                except:
                    pass
            raise

    def stop(self):
        logger.info("Stopping NatNet client...")
        self.connected = False
        if self.client:
            try:
                self.client.shutdown()
            except Exception as e:
                logger.warning(f"Error during NatNet client shutdown: {e}")

    def _on_rigid_body(self, body_id, position, orientation):
        now = time.time()
        with self._lock:
            if body_id not in self.bodies:
                self.bodies[body_id] = {'position': None, 'orientation': None,
                                        'count': 0, 'last_update': None}
            b = self.bodies[body_id]
            b['position'] = position
            b['orientation'] = orientation
            b['count'] += 1
            b['last_update'] = now
            self._last_rx_any = now
            count = b['count']

        if count == 1:
            logger.info(f"First position received for rigid body ID {body_id}")
            logger.info(f"  Position: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
        elif count % 1000 == 0:
            logger.debug(f"Rigid body ID {body_id}: {count} updates, "
                         f"pos=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})")

    def get_drone_position(self):
        with self._lock:
            body = self.bodies.get(5)
            if body and body['position'] is not None:
                return body['position'], body['orientation']
        return None, None

    def get_agv_position(self):
        with self._lock:
            body = self.bodies.get(1)
            if body and body['position'] is not None:
                return body['position'], body['orientation']
        return None, None

    def has_drone_position(self):
        with self._lock:
            body = self.bodies.get(5)
            return body is not None and body['position'] is not None

    def has_agv_position(self):
        with self._lock:
            body = self.bodies.get(1)
            return body is not None and body['position'] is not None

    def is_agv_lost(self, timeout_seconds=15.0):
        with self._lock:
            body = self.bodies.get(1)
            if body is None or body['last_update'] is None:
                return True  # Never had AGV position
            
            time_since_update = time.time() - body['last_update']
            return time_since_update > timeout_seconds

    def get_drone_status(self):
        with self._lock:
            body = self.bodies.get(5)
            if body is None:
                return {'has_position': False, 'position_count': 0}
                
            return {
                'has_position': body['position'] is not None,
                'position_count': body['count'],
                'current_position': body['position'],
                'current_orientation': body['orientation']
            }

    def get_connection_status(self):
        if not self.client:
            return {'connected': False, 'reason': 'Client not initialized'}
        
        with self._lock:
            available_bodies = list(self.bodies.keys())
            total_updates = sum(body['count'] for body in self.bodies.values())
        
        return {
            'connected': self.connected and self.client.connected(),
            'available_bodies': available_bodies,
            'total_updates': total_updates,
            'server_name': self.client.get_application_name() if self.client.connected() else 'Unknown',
            'natnet_version': self.client.get_nat_net_version_server() if self.client.connected() else [0,0,0,0]
        }

    def is_receiving_data(self, timeout_seconds=5.0):
        with self._lock:
            if self._last_rx_any is None:
                return False
            return (time.time() - self._last_rx_any) < timeout_seconds
