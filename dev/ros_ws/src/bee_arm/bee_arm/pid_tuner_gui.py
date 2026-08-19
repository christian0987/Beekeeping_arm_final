#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
#pid_tuner_gui.py
class PIDTunerGUI(Node):
    def __init__(self):
        super().__init__('pid_tuner_gui')

        # Publishers for arm and wrist PID parameters
        self.arm_pid_pub = self.create_publisher(Float64MultiArray, 'arm/pid_params', 10)
        self.wrist_pid_pub = self.create_publisher(Float64MultiArray, 'wrist/pid_params', 10)

        # Default parameters from the original code (15 values for 3 joints)
        # Format: for each joint: [Kp, Ki, Kd, offset, kp_pos]
        self.default_arm_params = [
            2.5, 0.1, 0.0, 150.0, 25.0,     # Joint 0
            2.5, 0.1, 0.0, 100.0, 5.0,     # Joint 1
            2.5, 0.1, 0.0, 150.0, 25.0      # Joint 2
        ]
        self.default_wrist_params = [
            2.5, 0.1, 0.0, 1500.0, 25.0,      # Joint 3
            2.5, 0.1, 0.0, 1500.0, 25.0,        # Joint 4
            1.0, 0.1, 0.0, 1500.0, 15.0      # Joint 5
        ]        
        # Store entry variable references
        self.arm_entries = []   # list of lists: 3 joints x 5 entries
        self.wrist_entries = []

        # Build GUI
        self.root = tk.Tk()
        self.root.title("PID Tuning - Arm & Wrist (6 joints)")
        self.root.geometry("550x500")
        
        self.create_widgets()

        # Start ROS spinning thread
        self.start_ros_spin()

    def create_widgets(self):
        # Main notebook style
        style = ttk.Style()
        style.configure("TLabel", font=("Arial", 10))
        style.configure("TLabelframe.Label", font=("Arial", 10, "bold"))

        # ----- Arm Frame -----
        arm_frame = ttk.LabelFrame(self.root, text="ARM PID Parameters (Joints 0-2)", padding=10)
        arm_frame.pack(side="top", fill="both", expand=True, padx=5, pady=5)

        # Create header labels
        header_frame = ttk.Frame(arm_frame)
        header_frame.pack(fill="x", pady=(0, 5))
        ttk.Label(header_frame, text="Joint ", width=8).pack(side="left", padx=2)
        ttk.Label(header_frame, text="Kp    ", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame, text="Ki    ", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame, text="Kd    ", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame, text="Offset", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame, text="Kp_pos", width=12).pack(side="left", padx=2)

        # Create rows for each arm joint
        for joint_idx in range(3):
            row_frame = ttk.Frame(arm_frame)
            row_frame.pack(fill="x", pady=3)

            # Joint label
            ttk.Label(row_frame, text=f"Joint {joint_idx}", width=6).pack(side="left", padx=2)

            # Entries for 5 parameters
            entries = []
            for param_idx, param_name in enumerate(["Kp", "Ki", "Kd", "Offset", "Kp_pos"]):
                var = tk.StringVar()
                # Set default value from default_arm_params
                default_val = self.default_arm_params[joint_idx*5 + param_idx]
                var.set(str(default_val))
                entry = ttk.Entry(row_frame, textvariable=var, width=10)
                entry.pack(side="left", padx=2)
                entries.append(var)
            self.arm_entries.append(entries)

        # Arm buttons
        arm_btn_frame = ttk.Frame(arm_frame)
        arm_btn_frame.pack(fill="x", pady=10)
        ttk.Button(arm_btn_frame, text="Send Arm PID", command=self.send_arm_pid).pack(side="left", padx=5)
        ttk.Button(arm_btn_frame, text="Reset Arm to Defaults", command=self.reset_arm_defaults).pack(side="left", padx=5)

        # ----- Wrist Frame -----
        wrist_frame = ttk.LabelFrame(self.root, text="WRIST PID Parameters (Joints 3-5)", padding=10)
        wrist_frame.pack(side="top", fill="both", expand=True, padx=5, pady=5)

        # Header
        header_frame2 = ttk.Frame(wrist_frame)
        header_frame2.pack(fill="x", pady=(0, 5))
        ttk.Label(header_frame2, text="Joint ", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame2, text="Kp    ", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame2, text="Ki    ", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame2, text="Kd    ", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame2, text="Offset", width=12).pack(side="left", padx=2)
        ttk.Label(header_frame2, text="Kp_pos", width=12).pack(side="left", padx=2)

        # Rows for wrist joints
        for joint_idx in range(3):
            row_frame = ttk.Frame(wrist_frame)
            row_frame.pack(fill="x", pady=3)

            ttk.Label(row_frame, text=f"Joint {joint_idx+3}", width=6).pack(side="left", padx=2)

            entries = []
            for param_idx in range(5):
                var = tk.StringVar()
                default_val = self.default_wrist_params[joint_idx*5 + param_idx]
                var.set(str(default_val))
                entry = ttk.Entry(row_frame, textvariable=var, width=10)
                entry.pack(side="left", padx=2)
                entries.append(var)
            self.wrist_entries.append(entries)

        # Wrist buttons
        wrist_btn_frame = ttk.Frame(wrist_frame)
        wrist_btn_frame.pack(fill="x", pady=10)
        ttk.Button(wrist_btn_frame, text="Send Wrist PID", command=self.send_wrist_pid).pack(side="left", padx=5)
        ttk.Button(wrist_btn_frame, text="Reset Wrist to Defaults", command=self.reset_wrist_defaults).pack(side="left", padx=5)

        # Global button to send both
        global_btn_frame = ttk.Frame(self.root)
        global_btn_frame.pack(side="bottom", pady=10)
        ttk.Button(global_btn_frame, text="Send All (Arm + Wrist)", command=self.send_all_pid).pack(side="left", padx=10)

    def get_arm_params_list(self):
        """Retrieve current values from arm entries as a list of 15 floats."""
        params = []
        for joint_entries in self.arm_entries:
            for var in joint_entries:
                try:
                    val = float(var.get().strip())
                except ValueError:
                    val = 0.0
                    self.get_logger().warn(f"Invalid number, using 0.0")
                params.append(val)
        return params

    def get_wrist_params_list(self):
        """Retrieve current values from wrist entries as a list of 15 floats."""
        params = []
        for joint_entries in self.wrist_entries:
            for var in joint_entries:
                try:
                    val = float(var.get().strip())
                except ValueError:
                    val = 0.0
                    self.get_logger().warn(f"Invalid number, using 0.0")
                params.append(val)
        return params

    def send_arm_pid(self):
        """Publish arm PID parameters to /arm/pid_params."""
        msg = Float64MultiArray()
        msg.data = self.get_arm_params_list()
        self.arm_pid_pub.publish(msg)
        self.get_logger().info(f"Published arm PID params: {msg.data}")

    def send_wrist_pid(self):
        """Publish wrist PID parameters to /wrist/pid_params."""
        msg = Float64MultiArray()
        msg.data = self.get_wrist_params_list()
        self.wrist_pid_pub.publish(msg)
        self.get_logger().info(f"Published wrist PID params: {msg.data}")

    def send_all_pid(self):
        self.send_arm_pid()
        self.send_wrist_pid()

    def reset_arm_defaults(self):
        """Reset all arm entry fields to default values."""
        for joint_idx in range(3):
            for param_idx in range(5):
                default_val = self.default_arm_params[joint_idx*5 + param_idx]
                self.arm_entries[joint_idx][param_idx].set(str(default_val))
        self.get_logger().info("Arm parameters reset to defaults")

    def reset_wrist_defaults(self):
        """Reset all wrist entry fields to default values."""
        for joint_idx in range(3):
            for param_idx in range(5):
                default_val = self.default_wrist_params[joint_idx*5 + param_idx]
                self.wrist_entries[joint_idx][param_idx].set(str(default_val))
        self.get_logger().info("Wrist parameters reset to defaults")

    def start_ros_spin(self):
        """Run ROS spinning in a background thread."""
        from threading import Thread
        def spin():
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        spin_thread = Thread(target=spin, daemon=True)
        spin_thread.start()

    def run(self):
        """Start the Tkinter main loop."""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        self.get_logger().info("Shutting down PID tuner GUI...")
        self.destroy_node()
        rclpy.shutdown()
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    gui = PIDTunerGUI()
    gui.run()

if __name__ == '__main__':
    main()