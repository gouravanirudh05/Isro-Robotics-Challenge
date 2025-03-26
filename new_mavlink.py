from pymavlink import mavutil
import time

class New_Mavlink_Control:
    def __init__(self, port='/dev/ttyUSB0', baud=57600):
        """
        Initialize connection with PX4 Pixhawk 2.4.8 via SIK3DR radio
        
        :param port: Serial port for SIK3DR radio (typically /dev/ttyUSB0 or COM port)
        :param baud: Baud rate (typically 57600 for SIK3DR)
        """
        try:
            self.connection = mavutil.mavlink_connection(port, baud=baud)
            
            # Wait for the first heartbeat with longer timeout
            start_time = time.time()
            while time.time() - start_time < 10:
                heartbeat = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
                if heartbeat:
                    print("Heartbeat from system (system %u component %u)" % 
                          (self.connection.target_system, self.connection.target_component))
                    break
            
            # PX4-specific mode mapping
            self.mode_mapping = {
                "MANUAL": 0,
                "ALTCTL": 1,
                "POSCTL": 2,
                "AUTO.MISSION": 3,
                "AUTO.LOITER": 4,
                "AUTO.RTL": 5,
                "ACRO": 6,
                "OFFBOARD": 7,
                "STABILIZED": 8,
                "RATTITUDE": 9,
                "AUTO.TAKEOFF": 10,
                "AUTO.LAND": 11,
                "GUIDED": 13,
                "AUTO.FOLLOW_TARGET": 12
            }
            
            self.home_position = None
        except Exception as e:
            print(f"Connection error: {e}")
            raise
    
    def get_drone_logs(self):
        """Receive MAVLink messages"""
        try:
            message = self.connection.recv_match(blocking=True)
            return message
        except Exception as e:
            print(f"Error receiving logs: {e}")
            return None
    
    def drone_arm(self):
        """
        Arm the drone with PX4-specific arming sequence
        
        :return: Boolean indicating successful arming
        """
        try:
            # Perform pre-arm safety checks
            # PX4 uses STABILIZED or ALTCTL for arming
            self.set_mode("STABILIZED")
            
            # Send arm command
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                1,  # arm
                21196,  # Magic code for force arming (PX4-specific)
                0, 0, 0, 0, 0
            )
            
            # Wait for ARM_STATUS or COMMAND_ACK
            start_time = time.time()
            while time.time() - start_time < 5:  # 5-second timeout
                ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
                if ack:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("Armed successfully")
                        return True
                    elif ack.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                        print("Arming rejected. Checking pre-arm conditions.")
                        # Optionally, print out specific pre-arm failure reasons
                        self.check_pre_arm_conditions()
                        return False
            
            print("Arming command timed out")
            return False
        
        except Exception as e:
            print(f"Arming error: {e}")
            return False

    def check_pre_arm_conditions(self):
        """
        Check and print out potential pre-arm failure reasons
        """
        try:
            # Request pre-arm checks
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            
            # Receive and print system status
            sys_status = self.connection.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
            if sys_status:
                print("System Status Checks:")
                print(f"Sensor health: {bin(sys_status.onboard_control_sensors_health)}")
                
            # Additional detailed checks
            print("\nPossible Pre-Arm Failure Reasons:")
            print("1. GPS lock not acquired")
            print("2. Compass calibration incomplete")
            print("3. Battery voltage too low")
            print("4. RC transmitter not connected")
            print("5. Safety switch engaged")
            print("6. Sensor calibration pending")
            
        except Exception as e:
            print(f"Pre-arm check error: {e}")
    
    def drone_disarm(self):
        """Disarm the drone"""
        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                0,  # disarm
                0, 0, 0, 0, 0, 0
            )
            
            # Wait for disarm acknowledgement
            ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Disarmed successfully")
                return True
            else:
                print("Disarming failed")
                return False
        except Exception as e:
            print(f"Disarming error: {e}")
            return False
    
    def set_mode(self, mode):
        """
        Set PX4 flight mode
        
        :param mode: Mode string (e.g., 'GUIDED', 'POSCTL')
        :return: Boolean indicating success
        """
        try:
            if mode not in self.mode_mapping:
                print(f"Unsupported mode: {mode}")
                return False
            
            # PX4 specific mode setting
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,  # confirmation
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.mode_mapping[mode],  # PX4 mode
                0, 0, 0, 0, 0
            )
            
            # Wait for mode change acknowledgement
            ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Mode change to {mode} acknowledged")
                return True
            else:
                print(f"Mode change to {mode} failed")
                return False
        except Exception as e:
            print(f"Mode change error: {e}")
            return False
    
    def takeoff(self, height):
        """
        Perform autonomous takeoff
        
        :param height: Takeoff altitude in meters
        :return: Boolean indicating successful takeoff
        """
        try:
            # Ensure in OFFBOARD mode for takeoff
            self.set_mode("OFFBOARD")
            
            # Arm drone if not already armed
            if not self.is_armed():
                self.drone_arm()
                time.sleep(1)
            
            # PX4 takeoff command
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # confirmation
                0,  # min pitch
                0, 0, 0,  # yaw, latitude, longitude
                0,  # min altitude
                height  # takeoff altitude
            )
            
            # Wait for takeoff acknowledgement
            ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Taking off to {height} meters")
                
                # Wait until reached target altitude
                start_time = time.time()
                while time.time() - start_time < 60:  # 60-second timeout
                    msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                    if msg:
                        current_alt = msg.relative_alt / 1000.0  # Convert to meters
                        print(f"Current altitude: {current_alt:.2f}m")
                        if abs(current_alt - height) < 0.5:  # Within 0.5m of target
                            print("Reached target altitude")
                            return True
                
                print("Altitude check timeout")
                return False
            else:
                print("Takeoff command failed")
                return False
        except Exception as e:
            print(f"Takeoff error: {e}")
            return False
    
    def move(self, x, y, z=None):
        """
        Move relative to current position
        
        :param x: Movement in x direction (meters)
        :param y: Movement in y direction (meters)
        :param z: Optional altitude change (meters)
        :return: Boolean indicating successful movement
        """
        try:
            # Ensure in OFFBOARD mode
            self.set_mode("OFFBOARD")
            
            # Get current position
            current_pos = self.get_position()
            if not current_pos:
                print("Failed to get current position")
                return False
            
            # Calculate target position
            target_x = current_pos[0] + x
            target_y = current_pos[1] + y
            target_z = z if z is not None else current_pos[2]
            
            # Send position command
            self.connection.mav.set_position_target_local_ned_send(
                0,  # timestamp
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # type mask (only position)
                target_x, target_y, target_z,
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0  # yaw, yaw_rate
            )
            
            # Wait for position reached
            start_time = time.time()
            while time.time() - start_time < 30:  # 30-second timeout
                current_pos = self.get_position()
                if current_pos:
                    dx = abs(current_pos[0] - target_x)
                    dy = abs(current_pos[1] - target_y)
                    print(f"Distance to target: dx={dx:.2f}m, dy={dy:.2f}m")
                    
                    if dx < 0.5 and dy < 0.5:
                        print("Reached target position")
                        return True
                time.sleep(0.5)
            
            print("Movement timed out")
            return False
        except Exception as e:
            print(f"Movement error: {e}")
            return False
    
    def land(self):
        """
        Land the drone
        
        :return: Boolean indicating successful landing
        """
        try:
            # Set LAND mode
            self.set_mode("AUTO.LAND")
            print("Landing initiated")
            
            # Wait for landing to complete
            start_time = time.time()
            while time.time() - start_time < 60:  # 60-second timeout
                msg = self.connection.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
                if msg and hasattr(msg, 'landed_state'):
                    if msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                        print("Successfully landed")
                        # Disarm after landing
                        self.drone_disarm()
                        return True
                time.sleep(0.5)
            
            print("Landing timed out")
            return False
        except Exception as e:
            print(f"Landing error: {e}")
            return False
    
    def get_position(self):
        """
        Get current drone position
        
        :return: Tuple of (x, y, z) coordinates or None
        """
        try:
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                return (msg.x, msg.y, -msg.z)  # z is down in NED frame
            return None
        except Exception as e:
            print(f"Position retrieval error: {e}")
            return None
    
    def is_armed(self):
        """
        Check if drone is armed
        
        :return: Boolean indicating arm status
        """
        try:
            heartbeat = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat:
                return (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            return False
        except Exception as e:
            print(f"Arm status check error: {e}")
            return False
    
    def get_battery_status(self):
        """
        Get detailed battery status
        
        :return: Dictionary of battery information
        """
        try:
            bat_msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1)
            if bat_msg:
                return {
                    'voltage': bat_msg.voltages[0] / 1000.0 if bat_msg.voltages[0] != 0xFFFF else 0,
                    'current': bat_msg.current_battery / 100.0 if bat_msg.current_battery != -1 else 0,
                    'remaining': bat_msg.battery_remaining,
                    'type': self.get_battery_type(bat_msg.type)
                }
            return None
        except Exception as e:
            print(f"Battery status error: {e}")
            return None
    
    def get_battery_type(self, battery_type):
        """
        Convert battery type code to readable string
        
        :param battery_type: Battery type code
        :return: Battery type as string
        """
        battery_types = {
            mavutil.mavlink.MAV_BATTERY_TYPE_UNKNOWN: "Unknown",
            mavutil.mavlink.MAV_BATTERY_TYPE_LIPO: "LiPo",
            mavutil.mavlink.MAV_BATTERY_TYPE_LIFE: "LiFe",
            mavutil.mavlink.MAV_BATTERY_TYPE_LION: "Li-Ion",
            mavutil.mavlink.MAV_BATTERY_TYPE_NIMH: "NiMH"
        }
        return battery_types.get(battery_type, "Other")
    
    def get_data(self):
        """
        Retrieve comprehensive drone data
        
        :return: Dictionary containing drone parameters
        """
        data = {}
        
        try:
            # Position and Velocity (LOCAL_POSITION_NED)
            pos_msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if pos_msg:
                data['position'] = {
                    'x': pos_msg.x,
                    'y': pos_msg.y,
                    'z': -pos_msg.z,  # Negate z as NED frame has z down
                    'vx': pos_msg.vx,
                    'vy': pos_msg.vy,
                    'vz': -pos_msg.vz
                }
            
            # Attitude (ATTITUDE)
            att_msg = self.connection.recv_match(type='ATTITUDE', blocking=True, timeout=1)
            if att_msg:
                data['attitude'] = {
                    'pitch': att_msg.pitch,
                    'roll': att_msg.roll,
                    'yaw': att_msg.yaw,
                    'pitch_speed': att_msg.pitchspeed,
                    'roll_speed': att_msg.rollspeed,
                    'yaw_speed': att_msg.yawspeed
                }
            
            # Battery Status
            bat_msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1)
            if bat_msg:
                data['battery'] = {
                    'voltage': bat_msg.voltages[0] / 1000.0 if bat_msg.voltages[0] != 0xFFFF else 0,
                    'current': bat_msg.current_battery / 100.0 if bat_msg.current_battery != -1 else 0,
                    'remaining': bat_msg.battery_remaining,
                    'type': self.get_battery_type(bat_msg.type)
                }
            
            # GPS Information
            gps_msg = self.connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if gps_msg:
                data['gps'] = {
                    'latitude': gps_msg.lat / 1e7,
                    'longitude': gps_msg.lon / 1e7,
                    'altitude': gps_msg.alt / 1000.0,
                    'satellites_visible': gps_msg.satellites_visible,
                    'fix_type': self.get_gps_fix_type(gps_msg.fix_type)
                }
            
            # System Status
            sys_msg = self.connection.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
            if sys_msg:
                data['system'] = {
                    'mode': self.get_current_mode(),
                    'armed': self.is_armed(),
                    'battery_voltage': sys_msg.voltage_battery / 1000.0,
                    'drop_rate_comm': sys_msg.drop_rate_comm,
                    'errors_comm': sys_msg.errors_comm
                }
            
            return data
        
        except Exception as e:
            print(f"Error retrieving drone data: {e}")
            return {}

    def get_gps_fix_type(self, fix_type):
        """
        Convert GPS fix type to readable string
        
        :param fix_type: GPS fix type code
        :return: GPS fix type as string
        """
        gps_fix_types = {
            0: "No GPS",
            1: "No Fix",
            2: "2D Fix",
            3: "3D Fix",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed"
        }
        return gps_fix_types.get(fix_type, "Unknown")

    def get_current_mode(self):
        """
        Get current flight mode
        
        :return: Current mode as string
        """
        try:
            heartbeat = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat:
                # Reverse lookup mode
                for mode_name, mode_value in self.mode_mapping.items():
                    if mode_value == heartbeat.custom_mode:
                        return mode_name
            return "Unknown"
        except Exception as e:
            print(f"Error getting current mode: {e}")
            return "Unknown"