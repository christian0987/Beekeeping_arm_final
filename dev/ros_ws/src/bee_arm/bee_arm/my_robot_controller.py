#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from subprocess import Popen
import tkinter as tk
from tkinter import ttk
import threading
import time

class MyRobotController(Node):
    def __init__(self):
        super().__init__('my_robot_controller')
        
        # Publishers pour le robot réel
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)
        
        # Lance joint_state_publisher_gui dans un processus séparé
        self.jsp_process = Popen([
            'ros2', 'run', 'joint_state_publisher_gui', 'joint_state_publisher_gui'
        ])
        
        # Attendre que joint_state_publisher_gui soit prêt
        time.sleep(2)
        
        # Subscriber pour écouter /joint_states (publié par joint_state_publisher_gui)
        self.sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        
        # Interface supplémentaire
        self.create_extra_gui()
        
        self.get_logger().info("MyRobotController demarre - joint_state_publisher_gui lance")
        self.get_logger().info("Deplace les sliders pour commander le robot reel ET la simulation")
    
    def joint_state_callback(self, msg):
        """Convertit /joint_states en commandes pour le robot réel"""
        # Trouve les index des joints
        arm_indices = []
        wrist_indices = []
        
        for i, name in enumerate(msg.name):
            if 'base_turn' in name or name == 'robot_arm_base_turn':
                arm_indices.append((i, 0))
            elif 'turn_joint' in name or name == 'robot_arm_turn_joint':
                arm_indices.append((i, 1))
            elif 'up_joint' in name or name == 'robot_arm_up_joint':
                arm_indices.append((i, 2))
            elif 'front_joint' in name or name == 'robot_arm_front_joint':
                wrist_indices.append((i, 0))
            elif 'down_joint' in name or name == 'robot_arm_down_joint':
                wrist_indices.append((i, 1))
            elif 'hand_joint' in name or name == 'robot_hand_joint':
                wrist_indices.append((i, 2))
        
        # Prepare les messages
        arm_pos = [0.0, 0.0, 0.0]
        wrist_pos = [0.0, 0.0, 0.0]
        
        for idx, pos_idx in arm_indices:
            if idx < len(msg.position):
                arm_pos[pos_idx] = msg.position[idx]
        
        for idx, pos_idx in wrist_indices:
            if idx < len(msg.position):
                wrist_pos[pos_idx] = msg.position[idx]
        
        # Publie vers le robot réel
        arm_msg = Float64MultiArray()
        arm_msg.data = arm_pos
        self.arm_pub.publish(arm_msg)
        
        wrist_msg = Float64MultiArray()
        wrist_msg.data = wrist_pos
        self.wrist_pub.publish(wrist_msg)
        
        self.get_logger().debug(f"Arm: {arm_pos}, Wrist: {wrist_pos}")
    
    def create_extra_gui(self):
        """Interface supplementaire pour quitter proprement"""
        self.root = tk.Tk()
        self.root.title("Controle Robot")
        self.root.geometry("250x150")
        
        ttk.Label(self.root, text="Robot Controller", font=("Arial", 12, "bold")).pack(pady=10)
        ttk.Label(self.root, text="Utilise les sliders pour commander\nle robot reel et la simulation").pack(pady=5)
        
        ttk.Button(self.root, text="Quitter", command=self.quit).pack(pady=20)
        
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
    
    def quit(self):
        self.get_logger().info("Arret du controller...")
        self.jsp_process.terminate()
        self.root.quit()
        rclpy.shutdown()
    
    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    controller = MyRobotController()
    
    # Thread pour ROS
    ros_thread = threading.Thread(target=lambda: rclpy.spin(controller), daemon=True)
    ros_thread.start()
    
    controller.run()

if __name__ == '__main__':
    main()
