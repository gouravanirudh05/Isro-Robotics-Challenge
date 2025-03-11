import tkinter as tk
from tkinter import ttk

class DroneFlightController:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Flight Controller")
        self.root.geometry("900x700")

        # Safe Spot Coordinates To be adjusted to work later
        self.safe_spot = (0.0, 0.0)
        
        # Top Bar
        self.top_frame = tk.Frame(self.root, bg="black", height=60)
        self.top_frame.pack(fill=tk.X)
        
        self.battery_label = tk.Label(self.top_frame, text="Battery: 23%", fg="red", bg="black", font=("Arial", 14, "bold"))
        self.battery_label.pack(side=tk.LEFT, padx=15)
        
        self.status_label = tk.Label(self.top_frame, text="Ready to Fly", fg="green", bg="black", font=("Arial", 14, "bold"))
        self.status_label.pack(side=tk.RIGHT, padx=15)
        
        # Map Display (Placeholder for now)
        self.map_frame = tk.Frame(self.root, bg="gray", height=500)
        self.map_frame.pack(fill=tk.BOTH, expand=True)
        self.map_label = tk.Label(self.map_frame, text="[Map Placeholder]", bg="gray", fg="white", font=("Arial", 16, "bold"))
        self.map_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        
        # Bottom Controls
        self.bottom_frame = tk.Frame(self.root, bg="black", height=120)
        self.bottom_frame.pack(fill=tk.X)
        
        self.altitude_label = tk.Label(self.bottom_frame, text="Altitude: 0.0 ft", fg="white", bg="black", font=("Arial", 12))
        self.altitude_label.pack(side=tk.LEFT, padx=15)
        
        self.speed_label = tk.Label(self.bottom_frame, text="Speed: 0.0 mph", fg="white", bg="black", font=("Arial", 12))
        self.speed_label.pack(side=tk.LEFT, padx=15)
        
        # Control Buttons
        self.control_frame = tk.Frame(self.bottom_frame, bg="black")
        self.control_frame.pack(side=tk.RIGHT, padx=15)
        
        button_style = {"font": ("Consolas", 14, "bold"), "width": 10, "height": 2}
        
        self.fly_button = tk.Button(self.control_frame, text="Fly", bg="green", fg="white", command=self.fly, **button_style)
        self.fly_button.pack(side=tk.LEFT, padx=10)
        
        self.plan_button = tk.Button(self.control_frame, text="Plan", bg="blue", fg="white", **button_style)
        self.plan_button.pack(side=tk.LEFT, padx=10)
        
        self.takeoff_button = tk.Button(self.control_frame, text="Takeoff", bg="orange", fg="white", command=self.takeoff, **button_style)
        self.takeoff_button.pack(side=tk.LEFT, padx=10)
        
        self.return_button = tk.Button(self.control_frame, text="Return", bg="red", fg="white", command=self.return_to_safe_spot, **button_style)
        self.return_button.pack(side=tk.LEFT, padx=10)
        
    def takeoff(self):
        print("Takeoff command sent")
        
    def return_to_safe_spot(self):
        lat, lon = self.safe_spot
        print(f"Returning to safe spot: {lat}, {lon}")
        
    def fly(self):
        print("Flying...")
        
if __name__ == "__main__":
    root = tk.Tk()
    app = DroneFlightController(root)
    root.mainloop()
