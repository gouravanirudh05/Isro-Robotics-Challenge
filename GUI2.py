import tkinter as tk
from tkinter import ttk
import time
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Import the Mavlink_Control class
from mavlink_commands import Mavlink_Control
from dummy_mavlink_controller import DummyMavlink_Control
from new_mavlink import New_Mavlink_Control

class DroneGUI:
    def __init__(self, root, mavlink_controller):
        self.root = root
        self.root.title("Drone Control Interface")
        self.root.geometry("800x600")
        self.controller = mavlink_controller

        
        # Create data update thread
        self.running = True
        self.update_interval = 500  # milliseconds
        
        # Create main frame for the GUI
        self.main_frame = ttk.Frame(root, padding="10")
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create sections
        self.create_status_section()
        self.create_control_section()
        self.create_data_section()
        self.create_visualization_section()
        
        # Start updating data
        self.update_data()
    
    def create_status_section(self):
        """Create the status display section"""
        status_frame = ttk.LabelFrame(self.main_frame, text="Status", padding="10")
        status_frame.pack(fill=tk.X, pady=5)
        
        # Connection status
        self.connection_var = tk.StringVar(value="Connected")
        ttk.Label(status_frame, text="Connection:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(status_frame, textvariable=self.connection_var).grid(row=0, column=1, sticky=tk.W)
        
        # Arm status
        self.arm_var = tk.StringVar(value="Disarmed")
        ttk.Label(status_frame, text="Arm Status:").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
        ttk.Label(status_frame, textvariable=self.arm_var).grid(row=0, column=3, sticky=tk.W)
        
        # Flight mode
        self.mode_var = tk.StringVar(value="STABILIZE")
        ttk.Label(status_frame, text="Flight Mode:").grid(row=0, column=4, sticky=tk.W, padx=(20, 0))
        ttk.Label(status_frame, textvariable=self.mode_var).grid(row=0, column=5, sticky=tk.W)
    
    def create_control_section(self):
        """Create the control buttons section"""
        control_frame = ttk.LabelFrame(self.main_frame, text="Controls", padding="10")
        control_frame.pack(fill=tk.X, pady=5)
        
        # Control buttons
        ttk.Button(control_frame, text="ARM", command=self.arm_drone).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(control_frame, text="DISARM", command=self.disarm_drone).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(control_frame, text="MANUAL MODE", command=self.set_manual_mode).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(control_frame, text="AUTO MODE", command=self.set_auto_mode).grid(row=0, column=3, padx=5, pady=5)
        ttk.Button(control_frame, text="TAKEOFF (5m)", command=lambda: self.takeoff(5)).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(control_frame, text="LAND", command=self.land).grid(row=1, column=1, padx=5, pady=5)
        
        # Movement controls
        move_frame = ttk.LabelFrame(control_frame, text="Movement", padding="5")
        move_frame.grid(row=1, column=2, columnspan=2, padx=5, pady=5, sticky=tk.W+tk.E)
        
        ttk.Button(move_frame, text="↑", command=lambda: self.move(1, 0)).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(move_frame, text="←", command=lambda: self.move(0, -1)).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(move_frame, text="→", command=lambda: self.move(0, 1)).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(move_frame, text="↓", command=lambda: self.move(-1, 0)).grid(row=2, column=1, padx=5, pady=5)
    
    def create_data_section(self):
        """Create the data display section"""
        data_frame = ttk.LabelFrame(self.main_frame, text="Drone Data", padding="10")
        data_frame.pack(fill=tk.X, pady=5)
        
        # Position data
        pos_frame = ttk.LabelFrame(data_frame, text="Position", padding="5")
        pos_frame.grid(row=0, column=0, padx=5, pady=5, sticky=tk.NW)
        
        self.pos_x_var = tk.StringVar(value="0.0 m")
        self.pos_y_var = tk.StringVar(value="0.0 m")
        self.pos_z_var = tk.StringVar(value="0.0 m")
        
        ttk.Label(pos_frame, text="X:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(pos_frame, textvariable=self.pos_x_var).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(pos_frame, text="Y:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(pos_frame, textvariable=self.pos_y_var).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(pos_frame, text="Height:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(pos_frame, textvariable=self.pos_z_var).grid(row=2, column=1, sticky=tk.W)
        
        # Velocity data
        vel_frame = ttk.LabelFrame(data_frame, text="Velocity", padding="5")
        vel_frame.grid(row=0, column=1, padx=5, pady=5, sticky=tk.NW)
        
        self.vel_x_var = tk.StringVar(value="0.0 m/s")
        self.vel_y_var = tk.StringVar(value="0.0 m/s")
        self.vel_z_var = tk.StringVar(value="0.0 m/s")
        
        ttk.Label(vel_frame, text="X Velocity:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(vel_frame, textvariable=self.vel_x_var).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(vel_frame, text="Y Velocity:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(vel_frame, textvariable=self.vel_y_var).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(vel_frame, text="Z Velocity:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(vel_frame, textvariable=self.vel_z_var).grid(row=2, column=1, sticky=tk.W)
        
        # Attitude data
        att_frame = ttk.LabelFrame(data_frame, text="Attitude", padding="5")
        att_frame.grid(row=0, column=2, padx=5, pady=5, sticky=tk.NW)
        
        self.pitch_var = tk.StringVar(value="0.0°")
        self.roll_var = tk.StringVar(value="0.0°")
        self.yaw_var = tk.StringVar(value="0.0°")
        
        ttk.Label(att_frame, text="Pitch:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(att_frame, textvariable=self.pitch_var).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(att_frame, text="Roll:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(att_frame, textvariable=self.roll_var).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(att_frame, text="Yaw:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(att_frame, textvariable=self.yaw_var).grid(row=2, column=1, sticky=tk.W)
        
        # Battery data
        bat_frame = ttk.LabelFrame(data_frame, text="Battery", padding="5")
        bat_frame.grid(row=0, column=3, padx=5, pady=5, sticky=tk.NW)
        
        self.voltage_var = tk.StringVar(value="0.0 V")
        self.current_var = tk.StringVar(value="0.0 A")
        self.percentage_var = tk.StringVar(value="0%")
        self.status_var = tk.StringVar(value="Unknown")
        
        ttk.Label(bat_frame, text="Voltage:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(bat_frame, textvariable=self.voltage_var).grid(row=0, column=1, sticky=tk.W)
        ttk.Label(bat_frame, text="Current:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(bat_frame, textvariable=self.current_var).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(bat_frame, text="Percentage:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(bat_frame, textvariable=self.percentage_var).grid(row=2, column=1, sticky=tk.W)
        ttk.Label(bat_frame, text="Status:").grid(row=3, column=0, sticky=tk.W)
        ttk.Label(bat_frame, textvariable=self.status_var).grid(row=3, column=1, sticky=tk.W)
    
    def create_visualization_section(self):
        """Create the visualization section with plots"""
        viz_frame = ttk.LabelFrame(self.main_frame, text="Visualization", padding="10")
        viz_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Create figure for plots
        self.fig = Figure(figsize=(8, 3), dpi=100)
        
        # Position plot
        self.pos_plot = self.fig.add_subplot(121)
        self.pos_plot.set_title("Position")
        self.pos_plot.set_xlabel("X (m)")
        self.pos_plot.set_ylabel("Y (m)")
        self.pos_plot.grid(True)
        self.pos_line, = self.pos_plot.plot([], [], 'ro-')
        self.pos_plot.set_xlim(-10, 10)
        self.pos_plot.set_ylim(-10, 10)
        
        # Altitude plot
        self.alt_plot = self.fig.add_subplot(122)
        self.alt_plot.set_title("Altitude")
        self.alt_plot.set_xlabel("Time (s)")
        self.alt_plot.set_ylabel("Height (m)")
        self.alt_plot.grid(True)
        self.alt_line, = self.alt_plot.plot([], [], 'bo-')
        self.alt_plot.set_xlim(0, 60)
        self.alt_plot.set_ylim(0, 10)
        
        # Position history
        self.pos_history = {'x': [], 'y': [], 'z': [], 'time': []}
        self.start_time = time.time()
        
        # Add the plot to the tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def update_data(self):
        """Update drone data at regular intervals"""
        if not self.running:
            return
            
        try:
            # Get data from the drone
            data = self.controller.get_data()
            
            if data:
                # Update position data
                if 'position' in data:
                    x, y, z = data['position']
                    self.pos_x_var.set(f"{x:.2f} m")
                    self.pos_y_var.set(f"{y:.2f} m")
                    self.pos_z_var.set(f"{z:.2f} m")
                    
                    # Update position history
                    current_time = time.time() - self.start_time
                    self.pos_history['x'].append(x)
                    self.pos_history['y'].append(y)
                    self.pos_history['z'].append(z)
                    self.pos_history['time'].append(current_time)
                    
                    # Limit history to last 60 seconds
                    if len(self.pos_history['time']) > 60:
                        self.pos_history['x'].pop(0)
                        self.pos_history['y'].pop(0)
                        self.pos_history['z'].pop(0)
                        self.pos_history['time'].pop(0)
                
                # Update velocity data
                if 'velocity' in data:
                    vx, vy, vz = data['velocity']
                    self.vel_x_var.set(f"{vx:.2f} m/s")
                    self.vel_y_var.set(f"{vy:.2f} m/s")
                    self.vel_z_var.set(f"{vz:.2f} m/s")
                
                # Update attitude data
                if 'attitude' in data:
                    pitch = data['attitude']['pitch']
                    roll = data['attitude']['roll']
                    yaw = data['attitude']['yaw']
                    # Convert radians to degrees
                    self.pitch_var.set(f"{pitch * 180 / 3.14159:.2f}°")
                    self.roll_var.set(f"{roll * 180 / 3.14159:.2f}°")
                    self.yaw_var.set(f"{yaw * 180 / 3.14159:.2f}°")
                
                # Update battery data
                if 'battery' in data:
                    voltage = data['battery']['voltage']
                    current = data['battery']['current']
                    percentage = data['battery']['percentage']
                    status = data['battery']['status']
                    
                    self.voltage_var.set(f"{voltage:.2f} V")
                    self.current_var.set(f"{current:.2f} A")
                    self.percentage_var.set(f"{percentage}%")
                    self.status_var.set(status)
                
                # Update arm status
                if self.controller.is_armed():
                    self.arm_var.set("Armed")
                else:
                    self.arm_var.set("Disarmed")
                
                # Update visualization
                self.update_visualization()
                
        except Exception as e:
            print(f"Error updating data: {e}")
        
        # Schedule the next update
        self.root.after(self.update_interval, self.update_data)
                                            
    def update_visualization(self):
        """Update the visualization plots"""
        # Update position plot
        if len(self.pos_history['x']) > 0:
            self.pos_line.set_data(self.pos_history['x'], self.pos_history['y'])
            self.pos_plot.set_xlim(min(self.pos_history['x']) - 1, max(self.pos_history['x']) + 1)
            self.pos_plot.set_ylim(min(self.pos_history['y']) - 1, max(self.pos_history['y']) + 1)
            
            # Update altitude plot
            self.alt_line.set_data(self.pos_history['time'], self.pos_history['z'])
            if len(self.pos_history['time']) > 0:
                self.alt_plot.set_xlim(0, max(self.pos_history['time']))
                self.alt_plot.set_ylim(0, max(max(self.pos_history['z']) + 1, 10))
        
        # Redraw the canvas
        self.canvas.draw()
    
    def arm_drone(self):
        """Arm the drone"""
        # Run in a separate thread to avoid blocking the GUI
        threading.Thread(target=self._arm_drone_thread).start()
    
    def _arm_drone_thread(self):
        """Thread function for arming the drone"""
        success = self.controller.drone_arm()
        if success:
            self.arm_var.set("Armed")
    
    def disarm_drone(self):
        """Disarm the drone"""
        threading.Thread(target=self._disarm_drone_thread).start()
    
    def _disarm_drone_thread(self):
        """Thread function for disarming the drone"""
        success = self.controller.drone_disarm()
        if success:
            self.arm_var.set("Disarmed")
    
    def set_manual_mode(self):
        """Set manual mode"""
        threading.Thread(target=self._set_manual_mode_thread).start()
    
    def _set_manual_mode_thread(self):
        """Thread function for setting manual mode"""
        success = self.controller.manual_mode()
        if success:
            self.mode_var.set("STABILIZE")
    
    def set_auto_mode(self):
        """Run auto mode sequence"""
        threading.Thread(target=self._set_auto_mode_thread).start()
    
    def _set_auto_mode_thread(self):
        """Thread function for running auto mode sequence"""
        self.controller.auto_mode()
    
    def takeoff(self, height):
        """Take off to specified height"""
        threading.Thread(target=lambda: self._takeoff_thread(height)).start()
    
    def _takeoff_thread(self, height):
        """Thread function for takeoff"""
        success = self.controller.takeoff(height)
        if success:
            self.mode_var.set("GUIDED")
            self.arm_var.set("Armed")
    
    def land(self):
        """Land the drone"""
        threading.Thread(target=self._land_thread).start()
    
    def _land_thread(self):
        """Thread function for landing"""
        success = self.controller.land()
        if success:
            self.mode_var.set("LAND")
            self.arm_var.set("Disarmed")
    
    def move(self, x, y):
        """Move the drone in x, y direction"""
        threading.Thread(target=lambda: self._move_thread(x, y)).start()
    
    def _move_thread(self, x, y):
        """Thread function for moving the drone"""
        self.controller.move(x, y)
    
    def update_data(self):
        """Update drone data at regular intervals"""
        if not self.running:
            return
        
        try:
            # Get data from the drone
            data = self.controller.get_data()
            
            if data:
                # Update position data
                if 'position' in data:
                    x, y, z = data['position']
                    self.pos_x_var.set(f"{x:.2f} m")
                    self.pos_y_var.set(f"{y:.2f} m")
                    self.pos_z_var.set(f"{z:.2f} m")
                    
                    # Update position history
                    current_time = time.time() - self.start_time
                    self.pos_history['x'].append(x)
                    self.pos_history['y'].append(y)
                    self.pos_history['z'].append(z)
                    self.pos_history['time'].append(current_time)
                    
                    # Limit history to last 60 seconds
                    if len(self.pos_history['time']) > 60:
                        self.pos_history['x'].pop(0)
                        self.pos_history['y'].pop(0)
                        self.pos_history['z'].pop(0)
                        self.pos_history['time'].pop(0)
                
                # Update velocity data
                if 'velocity' in data:
                    vx, vy, vz = data['velocity']
                    self.vel_x_var.set(f"{vx:.2f} m/s")
                    self.vel_y_var.set(f"{vy:.2f} m/s")
                    self.vel_z_var.set(f"{vz:.2f} m/s")
                
                # Update attitude data
                if 'attitude' in data:
                    pitch = data['attitude']['pitch']
                    roll = data['attitude']['roll']
                    yaw = data['attitude']['yaw']
                    # Convert radians to degrees
                    self.pitch_var.set(f"{pitch * 180 / 3.14159:.2f}°")
                    self.roll_var.set(f"{roll * 180 / 3.14159:.2f}°")
                    self.yaw_var.set(f"{yaw * 180 / 3.14159:.2f}°")
                
                # Update battery data
                if 'battery' in data:
                    voltage = data['battery']['voltage']
                    current = data['battery']['current']
                    percentage = data['battery']['percentage'] 
                    status = data['battery']['status']
                    
                    self.voltage_var.set(f"{voltage:.2f} V")
                    self.current_var.set(f"{current:.2f} A")
                    self.percentage_var.set(f"{percentage}%")
                    self.status_var.set(status)
                
                # Update flight mode
                if self.controller.is_armed():
                    self.arm_var.set("Armed")
                else:
                    self.arm_var.set("Disarmed")
                
                # Update visualization
                self.update_visualization()
        except Exception as e:
            print(f"Error updating data: {e}")
        
        # Schedule the next update
        self.root.after(self.update_interval, self.update_data)
    
    def on_closing(self):
        """Handle window closing"""
        self.running = False
        self.root.destroy()

def main():
    """Main function to start the GUI"""
    # Create the Mavlink controller
    mavlink_controller = New_Mavlink_Control(port="/dev/ttyUSB0",baud=57600) #Uncomment for actual working
    #mavlink_controller = DummyMavlink_Control(14551)
    
    # Create the GUI
    root = tk.Tk()
    app = DroneGUI(root, mavlink_controller)
    
    # Set up the closing protocol
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Start the GUI
    root.mainloop()

if __name__ == "__main__":
    main()
