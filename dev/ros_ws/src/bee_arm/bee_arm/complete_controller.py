#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import tkinter as tk
from tkinter import ttk
import threading

class CompleteRobotController(Node):
    def __init__(self):
        super().__init__('complete_robot_controller')
        
        # Publishers pour le robot réel
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)
        
        # Publisher pour MoveIt (simulation) - pour les sliders
        self.sim_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber pour la trajectoire planifiée par MoveIt
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Subscriber pour l'état des joints (sliders)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribers pour l'état du robot réel
        self.arm_state_sub = self.create_subscription(Float64MultiArray, 'arm/joint_state', self.arm_state_callback, 10)
        self.wrist_state_sub = self.create_subscription(Float64MultiArray, 'wrist/joint_state', self.wrist_state_callback, 10)
        
        # Variables
        self.arm_current = [0.0, 0.0, 0.0]
        self.wrist_current = [0.0, 0.0, 0.0]
        self.arm_desired = [0.0, 0.0, 0.0]
        self.wrist_desired = [0.0, 0.0, 0.0]
        
        # GUI
        self.root = tk.Tk()
        self.root.title("Robot Controller - MoveIt Planification + Sliders")
        self.root.geometry("600x500")
        
        self.create_widgets()
        
        # Timer pour mise à jour de l'affichage
        self.timer = self.create_timer(0.1, self.update_display)
        
        self.get_logger().info("Complete Robot Controller demarre")
        self.get_logger().info("-> Les sliders de joint_state_publisher_gui commandent la simulation")
        self.get_logger().info("-> La planification MoveIt (Plan & Execute) commande le robot reel")
    
    def trajectory_callback(self, msg):
        """Reçoit la trajectoire planifiée par MoveIt et l'envoie au robot réel"""
        if not msg.points:
            return
        
        # Prend le dernier point de la trajectoire (position cible)
        target = msg.points[-1]
        arm_pos = [0.0, 0.0, 0.0]
        wrist_pos = [0.0, 0.0, 0.0]
        
        # Convertit les positions (6 premiers joints)
        for i in range(min(3, len(target.positions))):
            arm_pos[i] = target.positions[i]
        for i in range(3, min(6, len(target.positions))):
            wrist_pos[i-3] = target.positions[i]
        
        # Publie vers le robot réel
        self.arm_pub.publish(Float64MultiArray(data=arm_pos))
        self.wrist_pub.publish(Float64MultiArray(data=wrist_pos))
        
        # Met à jour les valeurs désirées pour l'affichage
        self.arm_desired = arm_pos
        self.wrist_desired = wrist_pos
        
        self.get_logger().info(f"MOVEIT PLANIFIE - Arm: {arm_pos}, Wrist: {wrist_pos}")
    
    def joint_state_callback(self, msg):
        """Reçoit l'état des joints des sliders et les envoie à la simulation"""
        # Cette méthode peut être utilisée pour afficher les valeurs des sliders
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                if 'base_turn' in name or name == 'robot_arm_base_turn':
                    self.arm_desired[0] = msg.position[i]
                elif 'turn_joint' in name or name == 'robot_arm_turn_joint':
                    self.arm_desired[1] = msg.position[i]
                elif 'up_joint' in name or name == 'robot_arm_up_joint':
                    self.arm_desired[2] = msg.position[i]
                elif 'front_joint' in name or name == 'robot_arm_front_joint':
                    self.wrist_desired[0] = msg.position[i]
                elif 'down_joint' in name or name == 'robot_arm_down_joint':
                    self.wrist_desired[1] = msg.position[i]
                elif 'hand_joint' in name or name == 'robot_hand_joint':
                    self.wrist_desired[2] = msg.position[i]
    
    def arm_state_callback(self, msg):
        """Met à jour la position actuelle du bras (retour capteur)"""
        if len(msg.data) >= 3:
            self.arm_current = msg.data[:3]
    
    def wrist_state_callback(self, msg):
        """Met à jour la position actuelle du poignet (retour capteur)"""
        if len(msg.data) >= 3:
            self.wrist_current = msg.data[:3]
    
    def update_display(self):
        """Met à jour l'affichage"""
        try:
            # Unites
            unit_arm0 = "mm"
            unit_arm1 = "deg"
            unit_arm2 = "mm"
            unit_wrist0 = "mm"
            unit_wrist1 = "mm"
            unit_wrist2 = "deg"
            
            # Met à jour l'affichage des valeurs désirées (target)
            self.target_arm0_label.config(text=f"target: {self.arm_desired[0]:.3f} {unit_arm0}")
            self.target_arm1_label.config(text=f"target: {self.arm_desired[1]:.3f} {unit_arm1}")
            self.target_arm2_label.config(text=f"target: {self.arm_desired[2]:.3f} {unit_arm2}")
            self.target_wrist0_label.config(text=f"target: {self.wrist_desired[0]:.3f} {unit_wrist0}")
            self.target_wrist1_label.config(text=f"target: {self.wrist_desired[1]:.3f} {unit_wrist1}")
            self.target_wrist2_label.config(text=f"target: {self.wrist_desired[2]:.3f} {unit_wrist2}")
            
            # Met à jour l'affichage des valeurs actuelles (cur)
            self.cur_arm0_label.config(text=f"cur: {self.arm_current[0]:.3f} {unit_arm0}")
            self.cur_arm1_label.config(text=f"cur: {self.arm_current[1]:.3f} {unit_arm1}")
            self.cur_arm2_label.config(text=f"cur: {self.arm_current[2]:.3f} {unit_arm2}")
            self.cur_wrist0_label.config(text=f"cur: {self.wrist_current[0]:.3f} {unit_wrist0}")
            self.cur_wrist1_label.config(text=f"cur: {self.wrist_current[1]:.3f} {unit_wrist1}")
            self.cur_wrist2_label.config(text=f"cur: {self.wrist_current[2]:.3f} {unit_wrist2}")
            
        except Exception as e:
            pass
    
    def create_widgets(self):
        """Cree l'interface graphique"""
        style = ttk.Style()
        style.configure("TLabel", font=("Arial", 10))
        style.configure("TLabelframe.Label", font=("Arial", 10, "bold"))
        
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        ttk.Label(main_frame, text="CONTROLE DU ROBOT", font=("Arial", 14, "bold")).pack(pady=10)
        
        # Info sur les sources de commande
        info_frame = ttk.Frame(main_frame)
        info_frame.pack(fill="x", pady=5)
        ttk.Label(info_frame, text="Deux facons de commander le robot reel:", font=("Arial", 10, "bold")).pack(anchor="w")
        ttk.Label(info_frame, text="1. Deplace les sliders de joint_state_publisher_gui -> simulation uniquement", font=("Arial", 9)).pack(anchor="w")
        ttk.Label(info_frame, text="2. Planifie une trajectoire dans MoveIt (Plan & Execute) -> robot reel", font=("Arial", 9)).pack(anchor="w")
        
        # ========== ARM FRAME ==========
        arm_frame = ttk.LabelFrame(main_frame, text="ARM (Joints 0,1,2)", padding=10)
        arm_frame.pack(fill="x", pady=10)
        
        # Joint 0
        frame0 = ttk.Frame(arm_frame)
        frame0.pack(fill="x", pady=3)
        ttk.Label(frame0, text="Joint 0 (Horizontal):", width=20, anchor="w").pack(side="left")
        self.target_arm0_label = ttk.Label(frame0, text="target: 0.000", width=25)
        self.target_arm0_label.pack(side="left", padx=5)
        self.cur_arm0_label = ttk.Label(frame0, text="cur: 0.000 mm", width=20, foreground="blue")
        self.cur_arm0_label.pack(side="left", padx=5)
        
        # Joint 1
        frame1 = ttk.Frame(arm_frame)
        frame1.pack(fill="x", pady=3)
        ttk.Label(frame1, text="Joint 1 (Rotation):", width=20, anchor="w").pack(side="left")
        self.target_arm1_label = ttk.Label(frame1, text="target: 0.000", width=25)
        self.target_arm1_label.pack(side="left", padx=5)
        self.cur_arm1_label = ttk.Label(frame1, text="cur: 0.000 deg", width=20, foreground="blue")
        self.cur_arm1_label.pack(side="left", padx=5)
        
        # Joint 2
        frame2 = ttk.Frame(arm_frame)
        frame2.pack(fill="x", pady=3)
        ttk.Label(frame2, text="Joint 2 (Vertical):", width=20, anchor="w").pack(side="left")
        self.target_arm2_label = ttk.Label(frame2, text="target: 0.000", width=25)
        self.target_arm2_label.pack(side="left", padx=5)
        self.cur_arm2_label = ttk.Label(frame2, text="cur: 0.000 mm", width=20, foreground="blue")
        self.cur_arm2_label.pack(side="left", padx=5)
        
        # ========== WRIST FRAME ==========
        wrist_frame = ttk.LabelFrame(main_frame, text="WRIST (Joints 3,4,5)", padding=10)
        wrist_frame.pack(fill="x", pady=10)
        
        # Joint 3
        frame3 = ttk.Frame(wrist_frame)
        frame3.pack(fill="x", pady=3)
        ttk.Label(frame3, text="Joint 3 (Horizontal):", width=20, anchor="w").pack(side="left")
        self.target_wrist0_label = ttk.Label(frame3, text="target: 0.000", width=25)
        self.target_wrist0_label.pack(side="left", padx=5)
        self.cur_wrist0_label = ttk.Label(frame3, text="cur: 0.000 mm", width=20, foreground="blue")
        self.cur_wrist0_label.pack(side="left", padx=5)
        
        # Joint 4
        frame4 = ttk.Frame(wrist_frame)
        frame4.pack(fill="x", pady=3)
        ttk.Label(frame4, text="Joint 4 (Vertical):", width=20, anchor="w").pack(side="left")
        self.target_wrist1_label = ttk.Label(frame4, text="target: 0.000", width=25)
        self.target_wrist1_label.pack(side="left", padx=5)
        self.cur_wrist1_label = ttk.Label(frame4, text="cur: 0.000 mm", width=20, foreground="blue")
        self.cur_wrist1_label.pack(side="left", padx=5)
        
        # Joint 5
        frame5 = ttk.Frame(wrist_frame)
        frame5.pack(fill="x", pady=3)
        ttk.Label(frame5, text="Joint 5 (Rotation):", width=20, anchor="w").pack(side="left")
        self.target_wrist2_label = ttk.Label(frame5, text="target: 0.000", width=25)
        self.target_wrist2_label.pack(side="left", padx=5)
        self.cur_wrist2_label = ttk.Label(frame5, text="cur: 0.000 deg", width=20, foreground="blue")
        self.cur_wrist2_label.pack(side="left", padx=5)
        
        # Bouton quitter
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(pady=15)
        ttk.Button(button_frame, text="Quitter", command=self.quit).pack()
    
    def quit(self):
        self.root.quit()
        rclpy.shutdown()
    
    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    controller = CompleteRobotController()
    
    # Thread pour ROS
    ros_thread = threading.Thread(target=lambda: rclpy.spin(controller), daemon=True)
    ros_thread.start()
    
    controller.run()

if __name__ == '__main__':
    main()
