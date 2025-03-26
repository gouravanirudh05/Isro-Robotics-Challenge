from pymavlink import mavutil
import time

class Mavlink_Control:
    def __init__(self, port=14551,baud=57600):
        self.connection = mavutil.mavlink_connection(port,baud=baud)
        # Wait for the first heartbeat
        self.connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % 
              (self.connection.target_system, self.connection.target_component))
        self.home_position = None
        
    def get_drone_logs(self):
        message = self.connection.recv_match(blocking=True)
        return message
    
    def drone_arm(self):
        # First set the mode to GUIDED
        self.set_mode("GUIDED")
        
        # Send arm command
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        
        # Wait for ACK
        ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Armed successfully")
            return True
        else:
            print("Arming failed")
            return False
    
    def drone_disarm(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)
        
        # Wait for ACK
        ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Disarmed successfully")
            return True
        else:
            print("Disarming failed")
            return False
    
    def set_mode(self, mode):
        """Set the specified flight mode for PX4"""
        mode_mapping = {
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
            "AUTO.FOLLOW_TARGET": 12,
            "GUIDED": 13
        }
        
        if mode not in mode_mapping:
            print(f"Unsupported mode: {mode}")
            return False
        
        # PX4 uses different mode setting method
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[mode],  # PX4 mode
            0, 0, 0, 0, 0
        )
        
        # Wait for ACK
        ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"Mode change to {mode} acknowledged")
            return True
        else:
            print(f"Mode change to {mode} failed")
            return False
    
    def takeoff(self, height):
        # Ensure in GUIDED or OFFBOARD mode for takeoff
        self.set_mode("OFFBOARD")
        
        # Arm drone if not already armed
        if not self.is_armed():
            self.drone_arm()
            time.sleep(1)
        
        # PX4 specific takeoff command
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # min pitch
            0, 0, 0,  # yaw angle, latitude, longitude
            0,  # minimum altitude
            height  # takeoff altitude
        )
        
        # Wait for ACK
        ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"Taking off to {height} meters")
            
            # Wait until reached target altitude
            reached_alt = False
            while not reached_alt:
                msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    current_alt = msg.relative_alt / 1000.0  # Convert to meters
                    print(f"Current altitude: {current_alt:.2f}m")
                    if abs(current_alt - height) < 0.5:  # Within 0.5m of target
                        reached_alt = True
            print("Reached target altitude")
            return True
        else:
            print("Takeoff command failed")
            return False
    
    def move(self, x, y):
        '''Move x, y meters in x and y directions relative to current position'''
        # Make sure we're in GUIDED mode
        self.set_mode("GUIDED")
        
        # Get current position
        current_pos = self.get_position()
        if not current_pos:
            print("Failed to get current position")
            return False
        
        # Send position command (using current altitude)
        target_x = current_pos[0] + x
        target_y = current_pos[1] + y
        target_z = current_pos[2]  # Maintain current altitude
        
        # Send SET_POSITION_TARGET_LOCAL_NED message
        self.connection.mav.set_position_target_local_ned_send(
            0,  # timestamp
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate frame
            0b0000111111111000,  # type mask (only position)
            target_x, target_y, target_z,  # x, y, z positions in meters
            0, 0, 0,  # x, y, z velocity
            0, 0, 0,  # x, y, z acceleration
            0, 0)  # yaw, yaw_rate
        
        # Wait until reached target position
        reached_pos = False
        timeout = time.time() + 30  # 30 second timeout
        
        while not reached_pos and time.time() < timeout:
            current_pos = self.get_position()
            if current_pos:
                dx = abs(current_pos[0] - target_x)
                dy = abs(current_pos[1] - target_y)
                print(f"Distance to target: dx={dx:.2f}m, dy={dy:.2f}m")
                
                if dx < 0.5 and dy < 0.5:  # Within 0.5m of target
                    reached_pos = True
            time.sleep(0.5)
        
        if reached_pos:
            print("Reached target position")
            return True
        else:
            print("Movement timed out")
            return False
    
    def land(self):
        ''' Go back to the takeoff point and land at the takeoff point '''
        # First return to home position if we stored it
        if self.home_position:
            # Get current position
            current_pos = self.get_position()
            if current_pos:
                # Move horizontally to above home position
                self.move(self.home_position[0] - current_pos[0], 
                         self.home_position[1] - current_pos[1])
        
        # Set LAND mode
        self.set_mode("LAND")
        print("Landing initiated")
        
        # Wait for landing to complete
        landed = False
        timeout = time.time() + 60  # 60 second timeout
        
        while not landed and time.time() < timeout:
            # Check if we're on the ground
            msg = self.connection.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
            if msg and hasattr(msg, 'landed_state'):
                if msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                    landed = True
                    print("Successfully landed")
            time.sleep(0.5)
        
        if landed:
            # Disarm after landing
            self.drone_disarm()
            return True
        else:
            print("Landing timed out")
            return False
    
    def manual_mode(self):
        '''Allow user to control drone manually'''
        # Switch to STABILIZE mode which allows manual control
        return self.set_mode("STABILIZE")
    
    def auto_mode(self):
        ''' Makes the drone take off, move it a little bit and makes it land back at the same take off spot'''
        self.takeoff(5)  # Take off to 5 meters
        time.sleep(5)  # Hover for 5 seconds
        self.move(0.5, 0)  # Move 0.5m forward
        time.sleep(5)  # Hover for 5 seconds
        self.land()  # Return and land
    
    def get_data(self):
        ''' Return data on the following parameters of the drone:
         - position (x,y,height)
         - vertical and horizontal velocity
         - pitch, yaw and roll of the drone
         - battery health and status
        '''
        data = {}
        
        # Get position and velocity
        pos_msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if pos_msg:
            data['position'] = (pos_msg.x, pos_msg.y, -pos_msg.z)  # z is down in NED frame, so negate
            data['velocity'] = (pos_msg.vx, pos_msg.vy, -pos_msg.vz)
        
        # Get attitude (pitch, roll, yaw)
        att_msg = self.connection.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att_msg:
            data['attitude'] = {
                'pitch': att_msg.pitch,
                'roll': att_msg.roll,
                'yaw': att_msg.yaw
            }
        
        # Get battery info
        bat_msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1)
        if bat_msg:
            data['battery'] = {
                'voltage': bat_msg.voltages[0] / 1000.0 if bat_msg.voltages[0] != 0xFFFF else 0,
                'current': bat_msg.current_battery / 100.0 if bat_msg.current_battery != -1 else 0,
                'percentage': bat_msg.battery_remaining,
                'status': self.get_battery_status_text(bat_msg.battery_function, bat_msg.type)
            }
        
        return data
    
    def get_battery_status_text(self, function, type):
        """Convert battery function and type to readable text"""
        function_text = {
            mavutil.mavlink.MAV_BATTERY_FUNCTION_UNKNOWN: "Unknown",
            mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL: "All",
            mavutil.mavlink.MAV_BATTERY_FUNCTION_PROPULSION: "Propulsion",
            mavutil.mavlink.MAV_BATTERY_FUNCTION_AVIONICS: "Avionics"
        }.get(function, "Other")
        
        type_text = {
            mavutil.mavlink.MAV_BATTERY_TYPE_UNKNOWN: "Unknown",
            mavutil.mavlink.MAV_BATTERY_TYPE_LIPO: "LiPo",
            mavutil.mavlink.MAV_BATTERY_TYPE_LIFE: "LiFe",
            mavutil.mavlink.MAV_BATTERY_TYPE_LION: "Li-Ion",
            mavutil.mavlink.MAV_BATTERY_TYPE_NIMH: "NiMH"
        }.get(type, "Other")
        
        return f"{function_text} ({type_text})"
    
    def get_position(self):
        """Get current position (x, y, z) in meters"""
        msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            return (msg.x, msg.y, -msg.z)  # z is down in NED frame, so negate
        return None
    
    def get_home_position(self):
        """Get and store home position"""
        # Request home position
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0, 0, 0, 0, 0, 0, 0, 0)
        
        # Wait for HOME_POSITION message
        msg = self.connection.recv_match(type='HOME_POSITION', blocking=True, timeout=3)
        if msg:
            # Convert to local coordinates (this is simplified)
            self.home_position = (0, 0, 0)  # Home is (0,0,0) in local frame
            print("Home position stored")
            return True
        else:
            print("Failed to get home position")
            return False
    
    def is_armed(self):
        """Check if drone is armed"""
        heartbeat = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if heartbeat:
            armed = (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            return armed
        return False