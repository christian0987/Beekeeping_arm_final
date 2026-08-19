#!/usr/bin/env python3
#go_to_home_safe.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class GoToHomeSafe(Node):
    """Safely move robot to predefined positions with smooth start then constant speed"""
    
    def __init__(self):
        super().__init__('go_to_home_safe')
        
        # Publishers
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)
        
        # Subscribers
        self.arm_state_sub = self.create_subscription(Float64MultiArray, 'arm/joint_state', self.arm_state_callback, 10)
        self.wrist_state_sub = self.create_subscription(Float64MultiArray, 'wrist/joint_state', self.wrist_state_callback, 10)
        
        # Store current positions
        self.current_arm_pos = None
        self.current_wrist_pos = None
        self.positions_received = False
        
        # ==============================================
        # JOINT LIMITS - AVEC VOS MODIFICATIONS
        # ==============================================
        self.arm_limits = {
            'j0': (-325.0, 325.0),   # mm
            'j1': (-45.0, 180.0),    # deg - 179° est OK ✅
            'j2': (-430.0, 370.0),   # mm - MODIFIÉ (était -550 à 250)
        }
        self.wrist_limits = {
            'j3': (-25.0, 85.0),     # mm
            'j4': (-240.0, 50.0),    # mm
            'j5': (-180.0, 180.0),   # deg
        }
        
        # ==============================================
        # POSITIONS PRÉDÉFINIES
        # ==============================================
        self.positions = {
            'home': (
                [0.0, 0.0, 0.0],      # arm: [j0, j1, j2]
                [0.0, 0.0, 0.0]       # wrist: [j3, j4, j5]
            ),
            'standby': (
                [0.0, 179.0, 0.0],    # arm: j1 = 179° ✅
                [0.0, 0.0, 0.0]
            ),
        }
        
        # Tolérance pour les vérifications (0.5° ou 0.5mm)
        self.limit_tolerance = 0.5
        
        # Paramètres de mouvement
        self.publish_rate = 30.0      # Hz
        self.accel_percent = 0.15     # 15% pour l'accélération
        
        self.get_logger().info("="*60)
        self.get_logger().info("🤖 GoToHomeSafe node started")
        self.get_logger().info(f"📌 Available positions: {list(self.positions.keys())}")
        self.get_logger().info(f"⚡ Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"📈 Acceleration: {self.accel_percent*100}% of duration")
        self.get_logger().info("="*60)
    
    def arm_state_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_arm_pos = [msg.data[0], msg.data[1], msg.data[2]]
            self.check_positions_received()
    
    def wrist_state_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_wrist_pos = [msg.data[0], msg.data[1], msg.data[2]]
            self.check_positions_received()
    
    def check_positions_received(self):
        if self.current_arm_pos is not None and self.current_wrist_pos is not None:
            self.positions_received = True
            self.get_logger().info(f"📊 Current positions: arm={self.current_arm_pos}, wrist={self.current_wrist_pos}")
    
    def wait_for_initial_positions(self, timeout=5.0):
        start = time.time()
        while not self.positions_received and (time.time() - start) < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.positions_received:
            self.get_logger().warn("⏰ Timeout. Using zeros as initial positions.")
            self.current_arm_pos = [0.0, 0.0, 0.0]
            self.current_wrist_pos = [0.0, 0.0, 0.0]
            self.positions_received = True
    
    def clamp_value(self, value, min_val, max_val):
        """Clamp une valeur entre min et max"""
        return max(min_val, min(value, max_val))
    
    def check_limits(self, arm_target, wrist_target):
        """Vérifie et corrige les limites avec tolérance"""
        tol = self.limit_tolerance
        corrected = False
        
        # Vérifier les limites du bras
        if not (self.arm_limits['j0'][0] - tol <= arm_target[0] <= self.arm_limits['j0'][1] + tol):
            self.get_logger().warn(f"⚠️ J0 target {arm_target[0]:.2f} mm corrigé")
            arm_target[0] = self.clamp_value(arm_target[0], self.arm_limits['j0'][0], self.arm_limits['j0'][1])
            corrected = True
            
        if not (self.arm_limits['j1'][0] - tol <= arm_target[1] <= self.arm_limits['j1'][1] + tol):
            self.get_logger().warn(f"⚠️ J1 target {arm_target[1]:.2f} deg corrigé")
            arm_target[1] = self.clamp_value(arm_target[1], self.arm_limits['j1'][0], self.arm_limits['j1'][1])
            corrected = True
            
        if not (self.arm_limits['j2'][0] - tol <= arm_target[2] <= self.arm_limits['j2'][1] + tol):
            self.get_logger().warn(f"⚠️ J2 target {arm_target[2]:.2f} mm corrigé")
            arm_target[2] = self.clamp_value(arm_target[2], self.arm_limits['j2'][0], self.arm_limits['j2'][1])
            corrected = True
        
        # Vérifier les limites du poignet
        if not (self.wrist_limits['j3'][0] - tol <= wrist_target[0] <= self.wrist_limits['j3'][1] + tol):
            self.get_logger().warn(f"⚠️ J3 target {wrist_target[0]:.2f} mm corrigé")
            wrist_target[0] = self.clamp_value(wrist_target[0], self.wrist_limits['j3'][0], self.wrist_limits['j3'][1])
            corrected = True
            
        if not (self.wrist_limits['j4'][0] - tol <= wrist_target[1] <= self.wrist_limits['j4'][1] + tol):
            self.get_logger().warn(f"⚠️ J4 target {wrist_target[1]:.2f} mm corrigé")
            wrist_target[1] = self.clamp_value(wrist_target[1], self.wrist_limits['j4'][0], self.wrist_limits['j4'][1])
            corrected = True
            
        if not (self.wrist_limits['j5'][0] - tol <= wrist_target[2] <= self.wrist_limits['j5'][1] + tol):
            self.get_logger().warn(f"⚠️ J5 target {wrist_target[2]:.2f} deg corrigé")
            wrist_target[2] = self.clamp_value(wrist_target[2], self.wrist_limits['j5'][0], self.wrist_limits['j5'][1])
            corrected = True
        
        if corrected:
            self.get_logger().info("✅ Valeurs corrigées automatiquement")
        
        return True
    
    def get_accelerated_fraction(self, t, accel_percent):
        """
        Calcule la fraction avec accélération au début puis vitesse constante
        t: progression entre 0 et 1
        accel_percent: pourcentage du mouvement consacré à l'accélération (0.15 = 15%)
        """
        if t <= 0:
            return 0
        if t >= 1:
            return 1
        
        # Phase d'accélération
        if t <= accel_percent:
            normalized_t = t / accel_percent
            return normalized_t * normalized_t * 0.5
        
        # Phase de vitesse constante
        else:
            accel_end_value = 0.5
            linear_t = (t - accel_percent) / (1 - accel_percent)
            return accel_end_value + (1 - accel_end_value) * linear_t
    
    def go_to_position(self, position_name, duration=30.0):
        """Move to a predefined position by name with acceleration then constant speed"""
        if position_name not in self.positions:
            self.get_logger().error(f"❌ Position '{position_name}' not found!")
            self.get_logger().info(f"Available positions: {list(self.positions.keys())}")
            return
        
        end_arm, end_wrist = self.positions[position_name]
        
        self.get_logger().info("="*60)
        self.get_logger().info(f"🎯 Moving to position: {position_name}")
        self.get_logger().info(f"📍 Target arm={end_arm}, wrist={end_wrist}")
        
        # Vérifier et corriger la cible
        self.check_limits(end_arm, end_wrist)
        
        # Exécuter le mouvement
        self._move_to_target(end_arm, end_wrist, duration, position_name)
    
    def _move_to_target(self, end_arm, end_wrist, duration, position_name=""):
        """Internal method to move to a target position with acceleration then constant speed"""
        self.wait_for_initial_positions()
        
        start_arm = self.current_arm_pos[:]
        start_wrist = self.current_wrist_pos[:]
        
        # Vérifier et corriger la cible
        self.check_limits(end_arm, end_wrist)
        
        accel_percent = self.accel_percent
        
        self.get_logger().info(f"⏱️ Moving to {position_name} in {duration}s")
        self.get_logger().info(f"📈 Acceleration phase: {accel_percent*100:.0f}% of total duration")
        self.get_logger().info(f"📍 From arm={start_arm}, wrist={start_wrist}")
        
        start_time = time.time()
        publish_period = 1.0 / self.publish_rate
        
        last_progress = -1
        
        while time.time() - start_time < duration and rclpy.ok():
            elapsed = time.time() - start_time
            raw_fraction = min(1.0, elapsed / duration)
            
            # Appliquer la courbe avec accélération progressive
            fraction = self.get_accelerated_fraction(raw_fraction, accel_percent)
            
            current_arm = [
                start_arm[i] + (end_arm[i] - start_arm[i]) * fraction
                for i in range(3)
            ]
            current_wrist = [
                start_wrist[i] + (end_wrist[i] - start_wrist[i]) * fraction
                for i in range(3)
            ]
            
            # Vérifier et corriger les limites
            self.check_limits(current_arm, current_wrist)
            
            # Publier les positions
            arm_msg = Float64MultiArray()
            arm_msg.data = current_arm
            self.arm_pub.publish(arm_msg)
            
            wrist_msg = Float64MultiArray()
            wrist_msg.data = current_wrist
            self.wrist_pub.publish(wrist_msg)
            
            # Afficher la progression
            progress = int(raw_fraction * 100)
            if progress >= last_progress + 10:
                self.get_logger().info(f"📊 Progress: {progress}% complete")
                last_progress = progress
            
            time.sleep(publish_period)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        # Final target
        arm_msg = Float64MultiArray()
        arm_msg.data = end_arm
        self.arm_pub.publish(arm_msg)
        
        wrist_msg = Float64MultiArray()
        wrist_msg.data = end_wrist
        self.wrist_pub.publish(wrist_msg)
        
        self.get_logger().info("="*60)
        self.get_logger().info(f"✅ Robot successfully reached {position_name} position")
        self.get_logger().info("="*60)
    
    def go_to_home(self, duration=30.0):
        """Convenience method: go to home position"""
        self.go_to_position('home', duration)
    
    def go_to_standby(self, duration=25.0):
        """Convenience method: go to standby position"""
        self.go_to_position('standby', duration)

def main(args=None):
    rclpy.init(args=args)
    node = GoToHomeSafe()
    
    try:
        # ==============================================
        # CHOISISSEZ LA POSITION DÉSIRÉE
        # ==============================================
        
        # Aller à la position home (COMMENTÉ)
        # node.go_to_home(duration=30.0)
        
        # Aller à la position standby (ACTIF)
        node.go_to_standby(duration=25.0)  # 179° ✅
        
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()