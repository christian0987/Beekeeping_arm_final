#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState  # [AJOUT] Pour MoveIt (simulation)
import tkinter as tk
from tkinter import ttk

#bee_arm_gui.py

class CombinedRobotGUI(Node):
    def __init__(self):
        super().__init__('combined_robot_gui')

        # Publishers
        self.arm_target_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_target_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)

        # [AJOUT] Publisher pour MoveIt (simulation) - envoie les positions sur /joint_states
        self.sim_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscribers
        self.arm_state_sub = self.create_subscription(Float64MultiArray, 'arm/joint_state', self.arm_state_callback, 10)
        self.wrist_state_sub = self.create_subscription(Float64MultiArray, 'wrist/joint_state', self.wrist_state_callback, 10)

        # GUI
        self.root = tk.Tk()
        self.root.title("Combined Robot Control (Arm + Wrist)")
        self.root.geometry("850x400")

        # Arm variables
        self.arm_joint0_var = tk.DoubleVar(value=0.0)
        self.arm_joint1_var = tk.DoubleVar(value=0.0)
        self.arm_joint2_var = tk.DoubleVar(value=0.0)

        # Wrist variables
        self.wrist_joint0_var = tk.DoubleVar(value=0.0)
        self.wrist_joint1_var = tk.DoubleVar(value=0.0)
        self.wrist_joint2_var = tk.DoubleVar(value=0.0)

        self.arm_current = [0.0, 0.0, 0.0]
        self.wrist_current = [0.0, 0.0, 0.0]
        self.arm_initialized = False
        self.wrist_initialized = False

        self.create_widgets()

    # [AJOUT] Nouvelle méthode - Publie les positions vers MoveIt (simulatio) via /joint_states
    def publish_to_simulation(self):
        """Publie les positions vers MoveIt (simulation) via /joint_states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'robot_arm_base_turn',
            'robot_arm_turn_joint',
            'robot_arm_up_joint',
            'robot_arm_front_joint',
            'robot_arm_down_joint',
            'robot_hand_joint',
            'finger_1_joint',
            'finger_2_joint'
        ]
        msg.position = [
            self.arm_joint0_var.get(),
            self.arm_joint1_var.get(),
            self.arm_joint2_var.get(),
            self.wrist_joint0_var.get(),
            self.wrist_joint1_var.get(),
            self.wrist_joint2_var.get(),
            0.0,  # finger_1
            0.0   # finger_2
        ]
        self.sim_pub.publish(msg)

    def create_slider_row(self, parent, variable, from_, to, left_text, right_text,
                          label_text, step, is_angular):
        """Create a slider row with target and current position labels."""
        row_frame = ttk.Frame(parent)
        row_frame.pack(fill="x", pady=5)

        ttk.Label(row_frame, text=label_text, width=22, anchor="w").pack(side="left", padx=5)
        ttk.Label(row_frame, text=left_text).pack(side="left", padx=2)

        slider = ttk.Scale(row_frame, from_=from_, to=to, orient="horizontal",
                           variable=variable)
        slider.pack(side="left", fill="x", expand=True, padx=5)

        ttk.Label(row_frame, text=right_text).pack(side="left", padx=2)

        # Target value (slider)
        target_label = ttk.Label(row_frame, text=f"{variable.get():.1f}", width=8)
        target_label.pack(side="left", padx=5)

        # Current position label (will be updated from state)
        current_label = ttk.Label(row_frame, text="cur: --", width=12, foreground="blue")
        current_label.pack(side="left", padx=5)

        def update_target_label(*args):
            target_label.config(text=f"{variable.get():.1f}")
        variable.trace_add("write", update_target_label)

        # Mouse wheel support
        def on_mousewheel(event):
            if event.delta:
                delta = 1 if event.delta > 0 else -1
            elif event.num == 4:
                delta = 1
            elif event.num == 5:
                delta = -1
            else:
                return
            new_val = variable.get() + delta * step
            new_val = max(from_, min(to, new_val))
            variable.set(new_val)

        slider.bind("<MouseWheel>", on_mousewheel)
        slider.bind("<Button-4>", on_mousewheel)
        slider.bind("<Button-5>", on_mousewheel)

        return target_label, current_label

    def create_widgets(self):
        ttk.Style().configure("TLabel", font=("Arial", 10))
        ttk.Style().configure("TButton", font=("Arial", 10))
        ttk.Style().configure("TLabelframe.Label", font=("Arial", 10, "bold"))

        # ========== ARM FRAME ==========
        arm_frame = ttk.LabelFrame(self.root, text="ARM (Joints 0,1,2)", padding=10)
        arm_frame.pack(fill="x", padx=10, pady=5)

        # Store current labels for arm
        self.arm_current_labels = []

        # Joint 0
        _, cur_label = self.create_slider_row(
            arm_frame, self.arm_joint0_var, -325, 325,
            "-325 mm", "325 mm", "Horizontal translation (mm)",
            step=5.0, is_angular=False)
        self.arm_current_labels.append(cur_label)

        # Joint 1
        _, cur_label = self.create_slider_row(
            arm_frame, self.arm_joint1_var, -45, 180,
            "-45°", "180°", "Rotation (deg)",
            step=5.0, is_angular=True)
        self.arm_current_labels.append(cur_label)

        # Joint 2
        _, cur_label = self.create_slider_row(
            #arm_frame, self.arm_joint2_var, -550, 250,
            arm_frame, self.arm_joint2_var, -430, 370,

            #"-550 mm", "250 mm", "Vertical translation (mm)",
            "-430 mm", "370 mm", "Vertical translation (mm)",

            step=5.0, is_angular=False)
        self.arm_current_labels.append(cur_label)

        # Optional: keep a combined feedback label (or remove it)
        self.arm_feedback_label = ttk.Label(arm_frame, text="", font=("Arial", 9))
        self.arm_feedback_label.pack(pady=5)

        # ========== WRIST FRAME ==========
        wrist_frame = ttk.LabelFrame(self.root, text="WRIST (Joints 3,4,5)", padding=10)
        wrist_frame.pack(fill="x", padx=10, pady=5)

        self.wrist_current_labels = []

        # Joint 3
        _, cur_label = self.create_slider_row(
            wrist_frame, self.wrist_joint0_var, -25, 85,
            "-25 mm", "85 mm", "Horizontal translation (mm)",
            step=5.0, is_angular=False)
        self.wrist_current_labels.append(cur_label)

        # Joint 4
        _, cur_label = self.create_slider_row(
            wrist_frame, self.wrist_joint1_var, -240, 50,
            "-240 mm", "50 mm", "Vertical translation (mm)",
            

            # wrist_frame, self.wrist_joint1_var, -10, 280,
            # "-10 mm", "280 mm", "Vertical translation (mm)",
            
            step=5.0, is_angular=False)
        self.wrist_current_labels.append(cur_label)

        # Joint 5
        _, cur_label = self.create_slider_row(
            wrist_frame, self.wrist_joint2_var, -180, 180,
            "-180°", "180°", "Rotation (deg)",
            step=5.0, is_angular=True)
        self.wrist_current_labels.append(cur_label)

        self.wrist_feedback_label = ttk.Label(wrist_frame, text="", font=("Arial", 9))
        self.wrist_feedback_label.pack(pady=5)

        # Buttons
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=10)
        ttk.Button(button_frame, text="Send All Targets", command=self.publish_all_targets).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Send Arm Only", command=self.publish_arm_target).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Send Wrist Only", command=self.publish_wrist_target).pack(side="left", padx=5)

        # Real‑time publishing
        self.arm_joint0_var.trace_add("write", lambda *args: self.publish_arm_target())
        self.arm_joint1_var.trace_add("write", lambda *args: self.publish_arm_target())
        self.arm_joint2_var.trace_add("write", lambda *args: self.publish_arm_target())
        self.wrist_joint0_var.trace_add("write", lambda *args: self.publish_wrist_target())
        self.wrist_joint1_var.trace_add("write", lambda *args: self.publish_wrist_target())
        self.wrist_joint2_var.trace_add("write", lambda *args: self.publish_wrist_target())

    # Publishing methods (unchanged)
    def publish_arm_target(self):
        msg = Float64MultiArray()
        msg.data = [self.arm_joint0_var.get(), self.arm_joint1_var.get(), self.arm_joint2_var.get()]
        self.arm_target_pub.publish(msg)
        self.publish_to_simulation()  # [AJOUT] Envoie aussi à MoveIt
        self.get_logger().debug(f"Published arm target: {msg.data}")

    def publish_wrist_target(self):
        msg = Float64MultiArray()
        msg.data = [self.wrist_joint0_var.get(), self.wrist_joint1_var.get(), self.wrist_joint2_var.get()]
        self.wrist_target_pub.publish(msg)
        self.publish_to_simulation()  # [AJOUT] Envoie aussi à MoveIt
        self.get_logger().debug(f"Published wrist target: {msg.data}")

    def publish_all_targets(self):
        self.publish_arm_target()
        self.publish_wrist_target()

    # State callbacks with per‑joint label updates
    def arm_state_callback(self, msg):
        if len(msg.data) >= 3:
            self.arm_current = msg.data[:3]
            # Update each joint's "cur:" label
            for i, label in enumerate(self.arm_current_labels):
                unit = "mm" if i != 1 else "deg"
                label.config(text=f"cur: {self.arm_current[i]:.1f} {unit}")
            # Optional: update combined feedback label
            # self.arm_feedback_label.config(
            #     text=f"Arm current: {self.arm_current[0]:.1f} mm, "
            #          f"{self.arm_current[1]:.1f} deg, {self.arm_current[2]:.1f} mm"
            # )
            if not self.arm_initialized:
                self.arm_initialized = True
                self.arm_joint0_var.set(self.arm_current[0])
                self.arm_joint1_var.set(self.arm_current[1])
                self.arm_joint2_var.set(self.arm_current[2])
                self.get_logger().info("Arm sliders initialized to current joint positions.")

    def wrist_state_callback(self, msg):
        if len(msg.data) >= 3:
            self.wrist_current = msg.data[:3]
            for i, label in enumerate(self.wrist_current_labels):
                unit = "mm" if i != 2 else "deg"
                label.config(text=f"cur: {self.wrist_current[i]:.1f} {unit}")
            # self.wrist_feedback_label.config(
            #     text=f"Wrist current: {self.wrist_current[0]:.1f} mm, "
            #          f"{self.wrist_current[1]:.1f} mm, {self.wrist_current[2]:.1f} deg"
            # )
            if not self.wrist_initialized:
                self.wrist_initialized = True
                self.wrist_joint0_var.set(self.wrist_current[0])
                self.wrist_joint1_var.set(self.wrist_current[1])
                self.wrist_joint2_var.set(self.wrist_current[2])
                self.get_logger().info("Wrist sliders initialized to current joint positions.")

    # ROS spinning and main loop
    def run(self):
        from threading import Thread
        def spin():
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        spin_thread = Thread(target=spin, daemon=True)
        spin_thread.start()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        self.get_logger().info("Shutting down combined GUI...")
        self.destroy_node()
        rclpy.shutdown()
        self.root.quit()

def main(args=None):
    rclpy.init(args=args)
    gui = CombinedRobotGUI()
    gui.run()

if __name__ == '__main__':
    main()