import time
import random
import math
import threading

class DummyMavlink_Control:
    def __init__(self, port=14551):
        print(f"Connecting to dummy drone on port {port}...")
        time.sleep(1)
        print("Heartbeat from system (system 1 component 1)")
        
        # Dummy drone state
        self.armed = False
        self.mode = "STABILIZE"
        self.altitude = 0.0
        self.position = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.attitude = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}
        self.battery = {
            'voltage': 12.6,
            'current': 0.0,
            'percentage': 100,
            'status': "Propulsion (LiPo)"
        }
        self.home_position = (0.0, 0.0, 0.0)
        
        # Variables for simulating movement
        self.target_position = None
        self.target_altitude = None
        self.moving = False
        self.taking_off = False
        self.landing = False
        
        # Start the simulation thread
        self.running = True
        self.sim_thread = threading.Thread(target=self._simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()
    
    def _simulation_loop(self):
        """Simulates drone behavior in a separate thread"""
        last_time = time.time()
        
        while self.running:
            # Calculate time delta
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Update position based on velocity
            x, y, z = self.position
            vx, vy, vz = self.velocity
            
            # Add some noise to make it more realistic
            if self.armed:
                vx += random.uniform(-0.05, 0.05)
                vy += random.uniform(-0.05, 0.05)
                vz += random.uniform(-0.01, 0.01)
                
                # Limit velocity
                vx = max(-5.0, min(5.0, vx))
                vy = max(-5.0, min(5.0, vy))
                vz = max(-2.0, min(2.0, vz))
            
            # Update position
            x += vx * dt
            y += vy * dt
            z += vz * dt
            
            # Update state
            self.position = (x, y, z)
            self.velocity = (vx, vy, vz)
            
            # Battery discharge simulation
            if self.armed:
                # Discharge more when moving
                speed = math.sqrt(vx**2 + vy**2 + vz**2)
                discharge_rate = 0.01 + speed * 0.005
                self.battery['current'] = 5.0 + speed * 2.0
                self.battery['percentage'] = max(0, self.battery['percentage'] - discharge_rate * dt)
                self.battery['voltage'] = 12.6 * (0.8 + 0.2 * (self.battery['percentage'] / 100))
            else:
                self.battery['current'] = 0.2
            
            # Simulate attitude changes
            if self.armed:
                # Pitch forward when moving forward
                target_pitch = -0.1 * vx
                target_roll = 0.1 * vy
                
                # Smooth attitude changes
                self.attitude['pitch'] += (target_pitch - self.attitude['pitch']) * 0.1
                self.attitude['roll'] += (target_roll - self.attitude['roll']) * 0.1
                self.attitude['yaw'] += random.uniform(-0.01, 0.01)
            
            # Handle takeoff
            if self.taking_off and self.target_altitude is not None:
                # Move up at 1 m/s
                if z < self.target_altitude:
                    self.velocity = (vx, vy, 1.0)
                else:
                    self.velocity = (vx, vy, 0.0)
                    self.taking_off = False
            
            # Handle landing
            if self.landing:
                if z > 0.1:
                    self.velocity = (0.0, 0.0, -0.5)
                else:
                    self.velocity = (0.0, 0.0, 0.0)
                    self.position = (x, y, 0.0)
                    self.landing = False
                    self.armed = False
            
            # Handle position targeting
            if self.moving and self.target_position is not None:
                tx, ty, tz = self.target_position
                dx, dy = tx - x, ty - y
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist > 0.5:
                    # Move towards target at 2 m/s max
                    speed = min(2.0, dist)
                    vx = dx / dist * speed
                    vy = dy / dist * speed
                    self.velocity = (vx, vy, vz)
                else:
                    self.velocity = (0.0, 0.0, vz)
                    self.moving = False
            
            # Sleep to reduce CPU usage
            time.sleep(0.05)
    
    def get_drone_logs(self):
        return "Dummy log message"
    
    def drone_arm(self):
        print("Arming drone...")
        time.sleep(0.5)
        self.armed = True
        print("Armed successfully")
        return True
    
    def drone_disarm(self):
        print("Disarming drone...")
        time.sleep(0.5)
        self.armed = False
        print("Disarmed successfully")
        return True
    
    def set_mode(self, mode):
        print(f"Setting mode to {mode}...")
        time.sleep(0.5)
        self.mode = mode
        print(f"Mode change to {mode} acknowledged")
        return True
    
    def takeoff(self, height):
        print(f"Taking off to {height} meters...")
        
        # Store home position
        self.get_home_position()
        
        # Set to GUIDED mode
        self.set_mode("GUIDED")
        
        # Arm if not armed
        if not self.is_armed():
            self.drone_arm()
        
        # Start takeoff
        self.target_altitude = height
        self.taking_off = True
        
        # Simulate takeoff delay
        alt = 0.0
        while alt < height - 0.5:
            alt = self.position[2]
            print(f"Current altitude: {alt:.2f}m")
            time.sleep(0.5)
        
        print("Reached target altitude")
        return True
    
    def move(self, x, y):
        print(f"Moving {x}m in X and {y}m in Y...")
        
        # Make sure we're in GUIDED mode
        self.set_mode("GUIDED")
        
        # Get current position
        current_x, current_y, current_z = self.position
        
        # Calculate target position
        target_x = current_x + x
        target_y = current_y + y
        
        # Set target and start moving
        self.target_position = (target_x, target_y, current_z)
        self.moving = True
        
        # Simulate movement delay
        dist = 1.0
        while dist > 0.5:
            current_x, current_y, _ = self.position
            dx = abs(current_x - target_x)
            dy = abs(current_y - target_y)
            dist = math.sqrt(dx**2 + dy**2)
            print(f"Distance to target: dx={dx:.2f}m, dy={dy:.2f}m")
            time.sleep(0.5)
        
        print("Reached target position")
        return True
    
    def land(self):
        print("Landing initiated...")
        
        # Return to home position first
        if self.home_position:
            current_x, current_y, _ = self.position
            home_x, home_y, _ = self.home_position
            self.move(home_x - current_x, home_y - current_y)
        
        # Set LAND mode
        self.set_mode("LAND")
        
        # Start landing
        self.landing = True
        
        # Simulate landing delay
        alt = self.position[2]
        while alt > 0.1:
            alt = self.position[2]
            print(f"Current altitude: {alt:.2f}m")
            time.sleep(0.5)
        
        print("Successfully landed")
        self.drone_disarm()
        return True
    
    def manual_mode(self):
        print("Switching to manual control...")
        return self.set_mode("STABILIZE")
    
    def auto_mode(self):
        print("Running auto flight sequence...")
        self.takeoff(5)
        time.sleep(5)
        self.move(0.5, 0)
        time.sleep(5)
        self.land()
        return True
    
    def get_data(self):
        """Get current drone data"""
        return {
            'position': self.position,
            'velocity': self.velocity,
            'attitude': self.attitude,
            'battery': self.battery
        }
    
    def get_battery_status_text(self, function, type):
        """Dummy battery status text"""
        return "Propulsion (LiPo)"
    
    def get_position(self):
        """Get current position"""
        return self.position
    
    def get_home_position(self):
        """Get and store home position"""
        print("Home position stored")
        return True
    
    def is_armed(self):
        """Check if drone is armed"""
        return self.armed