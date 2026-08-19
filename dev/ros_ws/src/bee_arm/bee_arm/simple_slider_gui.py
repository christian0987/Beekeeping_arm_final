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

class SimpleSliderGUI(Node):
    def __init__(self):
        super().__init__('simple_slider_gui')
        
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
        
        # Variables pour les positions actuelles et desirees
        self.arm_desired = [0.0, 0.0, 0.0]
        self.wrist_desired = [0.0, 0.0, 0.0]
        
        # GUI
        self.root = tk.Tk()
        self.root.title("Simple Slider Controller (Real + Simulation)")
        self.root.geometry("500x450")
        
        # Creation des widgets
        self.create_widgets()
        
        # Timer pour mettre a jour l'affichage
        self.timer = self.create_timer(0.1, self.update_display)
        
        self.get_logger().info("Simple Slider GUI demarre")
        self.get_logger().info("Deplace les sliders pour commander le robot reel ET la simulation")
    
    def joint_state_callback(self, msg):
        """Convertit /joint_states en commandes pour le robot réel"""
        arm_indices = []
        wrist_indices = []
        
        for i, name in enumerate(msg.name):
            if 'base_turn' in name or name == 'robot_arm_base_turn':
                arm_indices.append((i, 0))
                self.arm_desired[0] = msg.position[i]
            elif 'turn_joint' in name or name == 'robot_arm_turn_joint':
                arm_indices.append((i, 1))
                self.arm_desired[1] = msg.position[i]
            elif 'up_joint' in name or name == 'robot_arm_up_joint':
                arm_indices.append((i, 2))
                self.arm_desired[2] = msg.position[i]
            elif 'front_joint' in name or name == 'robot_arm_front_joint':
                wrist_indices.append((i, 0))
                self.wrist_desired[0] = msg.position[i]
            elif 'down_joint' in name or name == 'robot_arm_down_joint':
                wrist_indices.append((i, 1))
                self.wrist_desired[1] = msg.position[i]
            elif 'hand_joint' in name or name == 'robot_hand_joint':
                wrist_indices.append((i, 2))
                self.wrist_desired[2] = msg.position[i]
        
        # Prepare les messages pour le robot reel
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
    
    def update_display(self):
        """Met a jour l'affichage des valeurs desirees (target) avec 3 decimales"""
        try:
            # Formatage avec 3 decimales
            arm0_val = self.arm_desired[0]
            arm1_val = self.arm_desired[1]
            arm2_val = self.arm_desired[2]
            wrist0_val = self.wrist_desired[0]
            wrist1_val = self.wrist_desired[1]
            wrist2_val = self.wrist_desired[2]
            
            # Unites
            unit_arm0 = "mm"
            unit_arm1 = "deg"
            unit_arm2 = "mm"
            unit_wrist0 = "mm"
            unit_wrist1 = "mm"
            unit_wrist2 = "deg"
            
            # Met a jour les labels
            self.target_arm0_label.config(text=f"target: {arm0_val:.3f} {unit_arm0}")
            self.target_arm1_label.config(text=f"target: {arm1_val:.3f} {unit_arm1}")
            self.target_arm2_label.config(text=f"target: {arm2_val:.3f} {unit_arm2}")
            self.target_wrist0_label.config(text=f"target: {wrist0_val:.3f} {unit_wrist0}")
            self.target_wrist1_label.config(text=f"target: {wrist1_val:.3f} {unit_wrist1}")
            self.target_wrist2_label.config(text=f"target: {wrist2_val:.3f} {unit_wrist2}")
            
            self.cur_arm0_label.config(text=f"cur: {arm0_val:.3f} {unit_arm0}")
            self.cur_arm1_label.config(text=f"cur: {arm1_val:.3f} {unit_arm1}")
            self.cur_arm2_label.config(text=f"cur: {arm2_val:.3f} {unit_arm2}")
            self.cur_wrist0_label.config(text=f"cur: {wrist0_val:.3f} {unit_wrist0}")
            self.cur_wrist1_label.config(text=f"cur: {wrist1_val:.3f} {unit_wrist1}")
            self.cur_wrist2_label.config(text=f"cur: {wrist2_val:.3f} {unit_wrist2}")
            
        except Exception as e:
            self.get_logger().warn(f"Erreur mise a jour affichage: {e}")
    
    def create_widgets(self):
        """Cree l'interface graphique"""
        style = ttk.Style()
        style.configure("TLabel", font=("Arial", 10))
        style.configure("TLabelframe.Label", font=("Arial", 10, "bold"))
        
        # Frame principal
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ttk.Label(main_frame, text="CONTROLE DU ROBOT", font=("Arial", 14, "bold")).pack(pady=10)
        ttk.Label(main_frame, text="Deplace les sliders de la fenetre joint_state_publisher_gui", font=("Arial", 9)).pack()
        ttk.Label(main_frame, text="pour commander le robot reel ET la simulation", font=("Arial", 9)).pack(pady=(0,15))
        
        # ========== ARM FRAME ==========
        arm_frame = ttk.LabelFrame(main_frame, text="ARM (Joints 0,1,2)", padding=10)
        arm_frame.pack(fill="x", pady=5)
        
        # Joint 0
        frame0 = ttk.Frame(arm_frame)
        frame0.pack(fill="x", pady=3)
        ttk.Label(frame0, text="Joint 0 (Horizontal):", width=20, anchor="w").pack(side="left")
        self.target_arm0_label = ttk.Label(frame0, text="target: 0.000", width=18)
        self.target_arm0_label.pack(side="left", padx=5)
        self.cur_arm0_label = ttk.Label(frame0, text="cur: --", width=18, foreground="blue")
        self.cur_arm0_label.pack(side="left", padx=5)
        
        # Joint 1
        frame1 = ttk.Frame(arm_frame)
        frame1.pack(fill="x", pady=3)
        ttk.Label(frame1, text="Joint 1 (Rotation):", width=20, anchor="w").pack(side="left")
        self.target_arm1_label = ttk.Label(frame1, text="target: 0.000", width=18)
        self.target_arm1_label.pack(side="left", padx=5)
        self.cur_arm1_label = ttk.Label(frame1, text="cur: --", width=18, foreground="blue")
        self.cur_arm1_label.pack(side="left", padx=5)
        
        # Joint 2
        frame2 = ttk.Frame(arm_frame)
        frame2.pack(fill="x", pady=3)
        ttk.Label(frame2, text="Joint 2 (Vertical):", width=20, anchor="w").pack(side="left")
        self.target_arm2_label = ttk.Label(frame2, text="target: 0.000", width=18)
        self.target_arm2_label.pack(side="left", padx=5)
        self.cur_arm2_label = ttk.Label(frame2, text="cur: --", width=18, foreground="blue")
        self.cur_arm2_label.pack(side="left", padx=5)
        
        # ========== WRIST FRAME ==========
        wrist_frame = ttk.LabelFrame(main_frame, text="WRIST (Joints 3,4,5)", padding=10)
        wrist_frame.pack(fill="x", pady=5)
        
        # Joint 3
        frame3 = ttk.Frame(wrist_frame)
        frame3.pack(fill="x", pady=3)
        ttk.Label(frame3, text="Joint 3 (Horizontal):", width=20, anchor="w").pack(side="left")
        self.target_wrist0_label = ttk.Label(frame3, text="target: 0.000", width=18)
        self.target_wrist0_label.pack(side="left", padx=5)
        self.cur_wrist0_label = ttk.Label(frame3, text="cur: --", width=18, foreground="blue")
        self.cur_wrist0_label.pack(side="left", padx=5)
        
        # Joint 4
        frame4 = ttk.Frame(wrist_frame)
        frame4.pack(fill="x", pady=3)
        ttk.Label(frame4, text="Joint 4 (Vertical):", width=20, anchor="w").pack(side="left")
        self.target_wrist1_label = ttk.Label(frame4, text="target: 0.000", width=18)
        self.target_wrist1_label.pack(side="left", padx=5)
        self.cur_wrist1_label = ttk.Label(frame4, text="cur: --", width=18, foreground="blue")
        self.cur_wrist1_label.pack(side="left", padx=5)
        
        # Joint 5
        frame5 = ttk.Frame(wrist_frame)
        frame5.pack(fill="x", pady=3)
        ttk.Label(frame5, text="Joint 5 (Rotation):", width=20, anchor="w").pack(side="left")
        self.target_wrist2_label = ttk.Label(frame5, text="target: 0.000", width=18)
        self.target_wrist2_label.pack(side="left", padx=5)
        self.cur_wrist2_label = ttk.Label(frame5, text="cur: --", width=18, foreground="blue")
        self.cur_wrist2_label.pack(side="left", padx=5)
        
        # Bouton quitter
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(pady=15)
        ttk.Button(button_frame, text="Quitter", command=self.quit).pack()
    
    def quit(self):
        self.get_logger().info("Arret du controller...")
        self.jsp_process.terminate()
        self.root.quit()
        rclpy.shutdown()
    
    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui = SimpleSliderGUI()
    
    # Thread pour ROS
    ros_thread = threading.Thread(target=lambda: rclpy.spin(gui), daemon=True)
    ros_thread.start()
    
    gui.run()

if __name__ == '__main__':
    main()