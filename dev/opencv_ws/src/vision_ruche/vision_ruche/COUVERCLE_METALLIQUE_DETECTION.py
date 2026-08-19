#!/usr/bin/env python3
"""
Detection du couvercle + Asservissement complet :
- Touche 's' : sequence complete : alignement vertical (J0) -> alignement angulaire (J1) 
               -> re-alignement vertical (J0) -> descente/remontee adaptative de J2.
- Touche 'd' : masques de debug.
- Touche 'q' : quitter.
Utilise cinq masques HSV (orange, jaune, blanc, beige, jaune-vert).
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from collections import deque
import math
import time
import sys
import urdfpy
import tempfile
import os

# ============================================================================
# URDF integre
# ============================================================================
URDF_STRING = '''<?xml version="1.0" encoding="utf-8"?>
<robot name="beekeeping_robot">
  <link name="base_link"/>
  <!-- Roues (non utilisees) -->
  <link name="wheel_turn_1_Link"/>
  <joint name="wheel_turn_1_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_turn_1_Link"/><axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_1_Link"/>
  <joint name="wheel1" type="continuous">
    <parent link="wheel_turn_1_Link"/><child link="wheel_1_Link"/><axis xyz="-1 0 0"/>
  </joint>
  <link name="wheel_turn_2_Link"/>
  <joint name="wheel_turn_2_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_turn_2_Link"/><axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_2_Link"/>
  <joint name="wheel2" type="continuous">
    <parent link="wheel_turn_2_Link"/><child link="wheel_2_Link"/><axis xyz="-1 0 0"/>
  </joint>
  <link name="wheel_turn_3_Link"/>
  <joint name="wheel_turn_3_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_turn_3_Link"/><axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_3_Link"/>
  <joint name="wheel3" type="continuous">
    <parent link="wheel_turn_3_Link"/><child link="wheel_3_Link"/><axis xyz="1 0 0"/>
  </joint>
  <link name="wheel_turn_4_Link"/>
  <joint name="wheel_turn_4_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_turn_4_Link"/><axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_4_Link"/>
  <joint name="wheel4" type="continuous">
    <parent link="wheel_turn_4_Link"/><child link="wheel_4_Link"/><axis xyz="1 0 0"/>
  </joint>
  <!-- Bras -->
  <link name="robot_arm_base_turn_Link"/>
  <joint name="robot_arm_base_turn" type="prismatic">
    <parent link="base_link"/><child link="robot_arm_base_turn_Link"/><axis xyz="1 0 0"/>
    <limit lower="-0.1" upper="0.38" effort="100" velocity="1.0"/>
  </joint>
  <link name="robot_arm_turn_Link"/>
  <joint name="robot_arm_turn_joint" type="continuous">
    <parent link="robot_arm_base_turn_Link"/><child link="robot_arm_turn_Link"/><axis xyz="0 0 1"/>
  </joint>
  <link name="robot_arm_up_Link"/>
  <joint name="robot_arm_up_joint" type="prismatic">
    <parent link="robot_arm_turn_Link"/><child link="robot_arm_up_Link"/><axis xyz="0 0 1"/>
    <limit lower="-0.05" upper="1" effort="100" velocity="1.0"/>
  </joint>
  <link name="robot_arm_front_Link"/>
  <joint name="robot_arm_front_joint" type="prismatic">
    <parent link="robot_arm_up_Link"/><child link="robot_arm_front_Link"/><axis xyz="0 -1 0"/>
    <limit lower="0" upper="0.1" effort="100" velocity="1.0"/>
  </joint>
  <link name="robot_arm_down_Link"/>
  <joint name="robot_arm_down_joint" type="prismatic">
    <parent link="robot_arm_front_Link"/><child link="robot_arm_down_Link"/><axis xyz="0 0 1"/>
    <limit lower="-0.3" upper="0" effort="100" velocity="1.0"/>
  </joint>
  <link name="robot_hand_Link"/>
  <joint name="robot_hand_joint" type="continuous">
    <parent link="robot_arm_down_Link"/><child link="robot_hand_Link"/><axis xyz="0 0 1"/>
  </joint>
</robot>'''

# ============================================================================
# Charger l'URDF
# ============================================================================
with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
    f.write(URDF_STRING)
    urdf_path = f.name
robot = urdfpy.URDF.load(urdf_path)
os.unlink(urdf_path)
print("URDF charge (integre)")

def get_effector_pose(joints):
    cfg = {
        'robot_arm_base_turn': joints[0]/1000.0,
        'robot_arm_turn_joint': math.radians(joints[1]),
        'robot_arm_up_joint': joints[2]/1000.0,
        'robot_arm_front_joint': joints[3]/1000.0,
        'robot_arm_down_joint': joints[4]/1000.0,
        'robot_hand_joint': math.radians(joints[5]),
    }
    fk = robot.link_fk(cfg=cfg)
    for link in robot.links:
        if link.name == 'robot_hand_Link':
            pose = fk[link].copy()
            pose[:3, 3] *= 100   # m -> cm
            return pose
    raise ValueError("robot_hand_Link not found")

class DetectionAndControl(Node):
    def __init__(self):
        super().__init__('detection_control_eih')
        
        # Publishers / Subscribers
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)
        self.arm_state_sub = self.create_subscription(Float64MultiArray, 'arm/joint_state', self.arm_state_callback, 10)
        self.wrist_state_sub = self.create_subscription(Float64MultiArray, 'wrist/joint_state', self.wrist_state_callback, 10)
        
        self.current_arm_pos = None
        self.current_wrist_pos = None
        self.positions_received = False
        self.last_z_cam = None   # pour memoriser la derniere distance Z_cam
        
        # Chargement des calibrations
        try:
            calib_data = np.load("camera_calib.npz")
            self.cam_mat = calib_data["cam_mat"]
            self.dist_coeff = calib_data["dist_coeff"]
            self.get_logger().info("Calibration camera chargee")
        except FileNotFoundError:
            self.get_logger().error("camera_calib.npz introuvable")
            sys.exit(1)
        
        try:
            self.T_cam_effector = np.load("T_cam_effector.npy").copy()
            self.T_cam_effector[:3, 3] *= 100   # m -> cm
            self.get_logger().info("Matrice T_cam_effector chargee (convertie en cm)")
        except FileNotFoundError:
            self.get_logger().error("T_cam_effector.npy introuvable")
            sys.exit(1)
        
        # Parametres de detection du couvercle
        self.LARGEUR_REEL = 51.0
        self.HAUTEUR_REEL = 41.0
        self.object_points_3d = np.array([
            [-self.LARGEUR_REEL/2, -self.HAUTEUR_REEL/2, 0],
            [ self.LARGEUR_REEL/2, -self.HAUTEUR_REEL/2, 0],
            [ self.LARGEUR_REEL/2,  self.HAUTEUR_REEL/2, 0],
            [-self.LARGEUR_REEL/2,  self.HAUTEUR_REEL/2, 0]
        ], dtype=np.float32)
        
        # --- Seuils HSV pour les cinq plages ---
        # ORANGE
        self.ORANGE_BAS = np.array([0, 40, 40])
        self.ORANGE_HAUT = np.array([20, 255, 255])
        
        # JAUNE
        self.JAUNE_BAS = np.array([20, 30, 30])
        self.JAUNE_HAUT = np.array([60, 255, 255])
        
        # BLANC
        self.BLANC_BAS = np.array([0, 0, 200])
        self.BLANC_HAUT = np.array([180, 30, 255])
        
        # BEIGE
        self.BEIGE_BAS = np.array([10, 30, 120])
        self.BEIGE_HAUT = np.array([35, 90, 220])
        
        # NOUVEAU : JAUNE-VERT (valeurs trouvees avec l'outil HSV)
        # BAS = [17, 0, 179]  HAUT = [76, 255, 255]
        self.JAUNE_VERT_BAS = np.array([17, 0, 179])
        self.JAUNE_VERT_HAUT = np.array([76, 255, 255])

        self.KERNEL_SIZE = (15, 15)
        self.CLOSE_ITER = 2
        self.AREA_MIN = 100
        self.POLY_EPSILON_FACTOR = 0.02
        self.corner_buffer = deque(maxlen=5)
        
        # Limites de securite (reelles)
        self.arm_limits = {
            'j0': (-325.0, 325.0),
            'j1': (-45.0, 180.0),
            'j2': (-430.0, 370.0),
        }
        self.wrist_limits = {
            'j3': (-25.0, 85.0),
            'j4': (-240.0, 50.0),
            'j5': (-180.0, 180.0),
        }
        self.debug = False
        self.debug_window_open = False
        self.cap = None
        self.get_logger().info("DetectionAndControl (eye-in-hand) pret")
        self.get_logger().info("Detection par cinq masques HSV (orange + jaune + blanc + beige + jaune-vert)")
    
    # ============================================================
    # Callbacks
    # ============================================================
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
    
    # ============================================================
    # Fonctions de detection
    # ============================================================
    def remplir_trous_blanc(self, mask_blanc):
        """
        Remplit les trous noirs à l'intérieur des formes blanches.
        Utilise la fermeture morphologique pour combler les espaces.
        """
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        mask_blanc_ferme = cv2.morphologyEx(mask_blanc, cv2.MORPH_CLOSE, kernel)
        return mask_blanc_ferme
    
    def masque_total(self, image):
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Les 5 masques
        mask_orange = cv2.inRange(hsv, self.ORANGE_BAS, self.ORANGE_HAUT)
        mask_jaune = cv2.inRange(hsv, self.JAUNE_BAS, self.JAUNE_HAUT)
        mask_blanc = cv2.inRange(hsv, self.BLANC_BAS, self.BLANC_HAUT)
        mask_beige = cv2.inRange(hsv, self.BEIGE_BAS, self.BEIGE_HAUT)
        mask_jaune_vert = cv2.inRange(hsv, self.JAUNE_VERT_BAS, self.JAUNE_VERT_HAUT)  # NOUVEAU
        
        # Remplir les trous dans le masque blanc
        mask_blanc_rempli = self.remplir_trous_blanc(mask_blanc)
        
        # Fusion des 5 masques
        mask = cv2.bitwise_or(mask_orange, mask_jaune)
        mask = cv2.bitwise_or(mask, mask_blanc_rempli)
        mask = cv2.bitwise_or(mask, mask_beige)
        mask = cv2.bitwise_or(mask, mask_jaune_vert)  # NOUVEAU
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.KERNEL_SIZE)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask, mask_orange, mask_jaune, mask_blanc_rempli, mask_beige, mask_jaune_vert  # NOUVEAU
    
    def order_points_hg_hd_bd_bg(self, pts):
        rect = np.zeros((4,2), dtype=np.float32)
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect
    
    def plus_grand_quadrilatere(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < self.AREA_MIN:
            return None, None
        peri = cv2.arcLength(c, True)
        eps = self.POLY_EPSILON_FACTOR * peri
        approx = cv2.approxPolyDP(c, eps, True)
        max_iter = 5
        while len(approx) != 4 and max_iter > 0:
            eps *= 1.2
            approx = cv2.approxPolyDP(c, eps, True)
            max_iter -= 1
        if len(approx) != 4:
            return None, None
        pts = approx.reshape(4,2).astype(np.float32)
        return pts, c
    
    def smooth_corners(self, new_corners):
        self.corner_buffer.append(new_corners)
        return np.mean(self.corner_buffer, axis=0).astype(np.float32)
    
    def compute_angle(self, corners_int):
        hg = corners_int[0]
        hd = corners_int[1]
        dx = hd[0] - hg[0]
        dy = hd[1] - hg[1]
        angle = math.degrees(math.atan2(dy, dx))
        return angle
    
    def detect_cover(self, frame):
        mask_total, mask_orange, mask_jaune, mask_blanc, mask_beige, mask_jaune_vert = self.masque_total(frame)  # NOUVEAU
        quad_points, _ = self.plus_grand_quadrilatere(mask_total)
        if quad_points is None:
            return None, False, None, None, None, None, None, mask_total, mask_orange, mask_jaune, mask_blanc, mask_beige, mask_jaune_vert  # NOUVEAU
        if len(self.corner_buffer) == 0:
            for _ in range(self.corner_buffer.maxlen):
                self.corner_buffer.append(quad_points.astype(np.float32))
        corners = self.smooth_corners(quad_points.astype(np.float32))
        corners_fixed = self.order_points_hg_hd_bd_bg(corners)
        success, rvec, tvec = cv2.solvePnP(
            self.object_points_3d, corners_fixed,
            self.cam_mat, self.dist_coeff
        )
        if not success:
            return None, False, None, None, None, None, None, mask_total, mask_orange, mask_jaune, mask_blanc, mask_beige, mask_jaune_vert  # NOUVEAU
        x_cam, y_cam, z_cam = tvec[0][0], tvec[1][0], tvec[2][0]
        return corners_fixed, True, x_cam, y_cam, z_cam, rvec, tvec, mask_total, mask_orange, mask_jaune, mask_blanc, mask_beige, mask_jaune_vert  # NOUVEAU
    
    # ============================================================
    # Mouvements de base
    # ============================================================
    def move_j0(self, delta_mm):
        if self.current_arm_pos is not None:
            new = self.current_arm_pos.copy()
            new[0] += delta_mm
            if self.arm_limits['j0'][0] <= new[0] <= self.arm_limits['j0'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.arm_pub.publish(msg)
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
    
    def move_j1(self, delta_deg):
        if self.current_arm_pos is not None:
            new = self.current_arm_pos.copy()
            new[1] += delta_deg
            if self.arm_limits['j1'][0] <= new[1] <= self.arm_limits['j1'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.arm_pub.publish(msg)
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
    
    def move_j2(self, delta_mm):
        if self.current_arm_pos is not None:
            new = self.current_arm_pos.copy()
            new[2] += delta_mm
            if self.arm_limits['j2'][0] <= new[2] <= self.arm_limits['j2'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.arm_pub.publish(msg)
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
    
    def wait_for_positions(self, timeout=3.0):
        start = time.time()
        while not self.positions_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.positions_received:
            self.get_logger().warn("Position actuelle inconnue, utilisation de 0")
            self.current_arm_pos = [0.0, 0.0, 0.0]
            self.current_wrist_pos = [0.0, 0.0, 0.0]
            self.positions_received = True

    # ============================================================
    # ALIGNEMENT VERTICAL (J0) avec memorisation de Z_cam
    # ============================================================
    def align_vertical(self):
        """Asservit J0 avec affichage en temps reel, et memorise la derniere Z_cam."""
        self.wait_for_positions()
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("ERREUR CAMERA")
            return
        
        # 13 valeurs car detect_cover retourne 13 valeurs (5 masques)
        _, success, _, _, _, _, _, _, _, _, _, _, _ = self.detect_cover(frame)
        if not success:
            self.get_logger().warn("Aucun couvercle detecte.")
            return
        
        h, w = frame.shape[:2]
        centre_cam_x = w // 2
        centre_cam_y = h // 2
        
        speed = 90          # mm/s
        dt = 0.1
        step = speed * dt   # 9 mm par iteration
        deadband = 6.0      # pixels
        max_iter = 300
        z_cam = None        # pour stocker la derniere valeur
        
        self.get_logger().info(f"Alignement vertical : Step={step:.2f} mm, Deadband={deadband} px")
        
        for i in range(max_iter):
            ret, frame = self.cap.read()
            if not ret:
                break
            
            corners, success, x_cam, y_cam, z_cam, _, _, mask_total, mask_orange, mask_jaune, mask_blanc, mask_beige, mask_jaune_vert = self.detect_cover(frame)
            result = frame.copy()
            
            # Centre camera
            cv2.circle(result, (centre_cam_x, centre_cam_y), 8, (0, 165, 255), -1)
            cv2.line(result, (centre_cam_x - 30, centre_cam_y), (centre_cam_x + 30, centre_cam_y), (0, 165, 255), 2)
            cv2.line(result, (centre_cam_x, centre_cam_y - 30), (centre_cam_x, centre_cam_y + 30), (0, 165, 255), 2)
            cv2.putText(result, "CAM CENTER", (centre_cam_x + 15, centre_cam_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
            cv2.line(result, (0, centre_cam_y), (w, centre_cam_y), (0, 0, 255), 2)
            
            if success:
                corners_int = corners.astype(np.int32)
                cv2.polylines(result, [corners_int], True, (0, 255, 0), 2)
                centre_couv = np.mean(corners_int, axis=0).astype(int)
                cv2.circle(result, tuple(centre_couv), 8, (255, 0, 255), -1)
                cv2.circle(result, tuple(centre_couv), 10, (255, 255, 255), 2)
                cv2.putText(result, "COVER CENTER", (centre_couv[0] + 15, centre_couv[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                cv2.line(result, (0, centre_couv[1]), (w, centre_couv[1]), (255, 0, 0), 2)
                
                diff_y = centre_couv[1] - centre_cam_y
                couleur = (0, 255, 255) if diff_y > 0 else (255, 255, 0)
                cv2.putText(result, f"Diff Y: {diff_y:+d} px", (w-200, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, couleur, 1)
                
                # Angle pour info
                hg = corners_int[0]
                hd = corners_int[1]
                dx = hd[0] - hg[0]
                dy = hd[1] - hg[1]
                angle_deg = math.degrees(math.atan2(dy, dx))
                cv2.putText(result, f"Angle: {angle_deg:+.1f} deg", (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.line(result, tuple(hg), tuple(hd), (0, 255, 255), 2)
                
                cv2.putText(result, f"Iter: {i+1}/{max_iter}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(result, f"J0: {self.current_arm_pos[0]:.1f} mm", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                cv2.putText(result, "Aucun couvercle", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            cv2.putText(result, "q: quitter", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Detection + Asservissement", result)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Interruption demandee")
                break
            
            if success:
                err = centre_couv[1] - centre_cam_y
                if abs(err) < deadband:
                    self.get_logger().info(f"Alignement vertical atteint (err={err:+.1f} px)")
                    break
                if err > 0:
                    self.move_j0(-step)
                else:
                    self.move_j0(step)
            else:
                self.get_logger().warn("Couvercle perdu")
                break
            
            time.sleep(dt)
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            self.get_logger().warn("Alignement vertical : nombre max d'iterations atteint.")
        
        # Memoriser la derniere Z_cam si disponible
        if z_cam is not None:
            self.last_z_cam = z_cam
            self.get_logger().info(f"Derniere distance Z_cam enregistree : {z_cam:.1f} cm")
        
        cv2.waitKey(1)
        self.get_logger().info("Alignement vertical termine.")

    # ============================================================
    # ALIGNEMENT ANGULAIRE (J1)
    # ============================================================
    def align_angle(self, deadband_angle=1.0, speed_angle=50.0):
        """Asservit J1 pour annuler l'angle du couvercle par rapport a l'horizontale."""
        self.wait_for_positions()
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("ERREUR CAMERA")
            return
        
        # 13 valeurs car detect_cover retourne 13 valeurs (5 masques)
        _, success, _, _, _, _, _, _, _, _, _, _, _ = self.detect_cover(frame)
        if not success:
            self.get_logger().warn("Aucun couvercle detecte.")
            return
        
        h, w = frame.shape[:2]
        centre_cam_x = w // 2
        centre_cam_y = h // 2
        
        dt = 0.1
        step_deg = speed_angle * dt   # pas en degres
        max_iter = 300
        
        self.get_logger().info(f"Alignement angulaire : Step={step_deg:.2f} deg, Deadband={deadband_angle} deg")
        
        for i in range(max_iter):
            ret, frame = self.cap.read()
            if not ret:
                break
            
            corners, success, x_cam, y_cam, z_cam, _, _, mask_total, mask_orange, mask_jaune, mask_blanc, mask_beige, mask_jaune_vert = self.detect_cover(frame)
            result = frame.copy()
            
            # Centre camera
            cv2.circle(result, (centre_cam_x, centre_cam_y), 8, (0, 165, 255), -1)
            cv2.line(result, (centre_cam_x - 30, centre_cam_y), (centre_cam_x + 30, centre_cam_y), (0, 165, 255), 2)
            cv2.line(result, (centre_cam_x, centre_cam_y - 30), (centre_cam_x, centre_cam_y + 30), (0, 165, 255), 2)
            cv2.putText(result, "CAM CENTER", (centre_cam_x + 15, centre_cam_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
            cv2.line(result, (0, centre_cam_y), (w, centre_cam_y), (0, 0, 255), 2)
            
            if success:
                corners_int = corners.astype(np.int32)
                cv2.polylines(result, [corners_int], True, (0, 255, 0), 2)
                centre_couv = np.mean(corners_int, axis=0).astype(int)
                cv2.circle(result, tuple(centre_couv), 8, (255, 0, 255), -1)
                cv2.circle(result, tuple(centre_couv), 10, (255, 255, 255), 2)
                cv2.putText(result, "COVER CENTER", (centre_couv[0] + 15, centre_couv[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                cv2.line(result, (0, centre_couv[1]), (w, centre_couv[1]), (255, 0, 0), 2)
                
                # Calcul de l'angle
                hg = corners_int[0]
                hd = corners_int[1]
                dx = hd[0] - hg[0]
                dy = hd[1] - hg[1]
                angle_deg = math.degrees(math.atan2(dy, dx))
                
                cv2.putText(result, f"Angle: {angle_deg:+.1f} deg", (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.line(result, tuple(hg), tuple(hd), (0, 255, 255), 2)
                
                cv2.putText(result, f"Iter ang: {i+1}/{max_iter}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(result, f"J1: {self.current_arm_pos[1]:.1f} deg", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                cv2.putText(result, "Aucun couvercle", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            cv2.putText(result, "q: quitter", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Detection + Asservissement", result)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Interruption demandee")
                break
            
            if success:
                if abs(angle_deg) < deadband_angle:
                    self.get_logger().info(f"Alignement angulaire atteint (angle={angle_deg:+.1f} deg)")
                    break
                if angle_deg > 0:
                    self.move_j1(-step_deg)
                else:
                    self.move_j1(step_deg)
            else:
                self.get_logger().warn("Couvercle perdu")
                break
            
            time.sleep(dt)
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            self.get_logger().warn("Alignement angulaire : nombre max d'iterations atteint.")
        
        cv2.waitKey(1)
        self.get_logger().info("Alignement angulaire termine.")

    # ============================================================
    # DESCENTE / REMONTEE ADAPTATIVE DE J2
    # ============================================================
    def descente_remontee(self, distance_mm, duree=10.0, tolerance=2.0, timeout=15.0):
        """
        Descend J2 de distance_mm, attend la fin reelle du mouvement,
        puis reste en bas pendant duree secondes, puis remonte a J2=0.
        """
        self.wait_for_positions()
        
        if self.current_arm_pos is None:
            self.get_logger().error("Position de J2 inconnue")
            return
        
        j2_initial = self.current_arm_pos[2]
        j2_cible = j2_initial - distance_mm
        
        self.get_logger().info(f"Position initiale de J2 : {j2_initial:.1f} mm")
        self.get_logger().info(f"Position cible (en bas) : {j2_cible:.1f} mm")
        self.get_logger().info(f"Descente de J2 de {distance_mm:.1f} mm...")
        
        # 1. Envoyer la commande de descente
        self.move_j2(-distance_mm)
        
        # 2. Attendre que le mouvement soit reellement termine (avec timeout)
        self.get_logger().info("Attente de la fin du mouvement de descente...")
        start = time.time()
        while time.time() - start < timeout:
            if self.current_arm_pos is not None:
                erreur = abs(self.current_arm_pos[2] - j2_cible)
                if erreur < tolerance:
                    self.get_logger().info(f"Mouvement termine, position J2 = {self.current_arm_pos[2]:.1f} mm")
                    break
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        else:
            self.get_logger().warn(f"Mouvement non termine apres {timeout}s, position actuelle = {self.current_arm_pos[2] if self.current_arm_pos else '?'} mm")
        
        # 3. Attendre la duree demandee en bas (10s)
        self.get_logger().info(f"Attente de {duree} secondes en bas...")
        start_wait = time.time()
        while time.time() - start_wait < duree:
            elapsed = int(time.time() - start_wait)
            remaining = int(duree - elapsed)
            if remaining > 0 and (time.time() - start_wait) - elapsed < 0.1:
                self.get_logger().info(f"{remaining} secondes restantes")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        
        # 4. Remonter a la position initiale (J2 = 0)
        self.get_logger().info(f"Remontee de J2 de {distance_mm:.1f} mm vers J2 = {j2_initial:.1f} mm...")
        self.move_j2(distance_mm)
        
        time.sleep(0.5)
        self.get_logger().info("Descente/remontee terminee.")

    # ============================================================
    # SEQUENCE COMPLETE (avec calcul dynamique de la descente)
    # ============================================================
    def align_complet(self):
        
        """Enchaine alignement vertical, alignement angulaire, re-alignement vertical,
        puis descente/remontee adaptative pour arriver a 20 cm du couvercle."""
        self.get_logger().info("Debut de la sequence complete.")
        self.align_vertical()
        self.align_angle()
        self.align_vertical()
        
        # Calcul de la descente pour arriver a 20 cm du couvercle
        if self.last_z_cam is not None:
            cible_cm = 20.0
            distance_cm = self.last_z_cam - cible_cm
            distance_mm = distance_cm * 10  # conversion cm -> mm
            if distance_mm > 0:
                self.get_logger().info(f"Z_cam = {self.last_z_cam:.1f} cm, on descend de {distance_mm:.1f} mm pour arriver a {cible_cm} cm.")
                self.descente_remontee(distance_mm)
            else:
                self.get_logger().warn(f"Z_cam ({self.last_z_cam:.1f} cm) est deja <= {cible_cm} cm, pas de descente necessaire.")
        else:
            self.get_logger().warn("Aucune valeur de Z_cam disponible, impossible de faire la descente adaptative.")
        
        self.get_logger().info("Sequence complete terminee.")

    # ============================================================
    # Boucle principale
    # ============================================================
    def run(self):
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("Camera introuvable")
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.get_logger().info("\n=== SEQUENCE COMPLETE (J0 + J1 + J0 + descente adaptative J2) ===")
        self.get_logger().info("Appuyez sur 's' pour lancer la sequence complete")
        self.get_logger().info("Appuyez sur 'd' pour le masque de debug")
        self.get_logger().info("Appuyez sur 'q' pour quitter\n")
        
        debug_names = {
            'total': "Debug - Masque total",
            'orange': "Debug - Masque orange",
            'jaune': "Debug - Masque jaune",
            'blanc': "Debug - Masque blanc",
            'beige': "Debug - Masque beige",
            'jaune_vert': "Debug - Masque jaune-vert"  # NOUVEAU
        }
        for name in debug_names.values():
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 320, 240)
            idx = list(debug_names.values()).index(name)
            cv2.moveWindow(name, 700 + 330 * idx, 0)
        
        black = np.zeros((240, 320, 1), dtype=np.uint8)
        for name in debug_names.values():
            cv2.imshow(name, black)
        
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                break
            result = frame.copy()
            h, w = result.shape[:2]
            centre_image_x = w // 2
            centre_image_y = h // 2
            centre_image = (centre_image_x, centre_image_y)
            
            # Centre de la camera
            cv2.circle(result, centre_image, 8, (0, 165, 255), -1)
            cv2.line(result, (centre_image_x - 30, centre_image_y), (centre_image_x + 30, centre_image_y), (0, 165, 255), 2)
            cv2.line(result, (centre_image_x, centre_image_y - 30), (centre_image_x, centre_image_y + 30), (0, 165, 255), 2)
            cv2.putText(result, "CAM CENTER", (centre_image_x + 15, centre_image_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
            cv2.line(result, (0, centre_image_y), (w, centre_image_y), (0, 0, 255), 2)
            
            # Detection du couvercle
            corners, success, x_cam, y_cam, z_cam, rvec, tvec, mask_total, mask_orange, mask_jaune, mask_blanc, mask_beige, mask_jaune_vert = self.detect_cover(frame)
            
            # Debug des masques
            if self.debug:
                display_size = (320, 240)
                mask_total_disp = cv2.resize(mask_total, display_size, interpolation=cv2.INTER_NEAREST)
                mask_orange_disp = cv2.resize(mask_orange, display_size, interpolation=cv2.INTER_NEAREST)
                mask_jaune_disp = cv2.resize(mask_jaune, display_size, interpolation=cv2.INTER_NEAREST)
                mask_blanc_disp = cv2.resize(mask_blanc, display_size, interpolation=cv2.INTER_NEAREST)
                mask_beige_disp = cv2.resize(mask_beige, display_size, interpolation=cv2.INTER_NEAREST)
                mask_jaune_vert_disp = cv2.resize(mask_jaune_vert, display_size, interpolation=cv2.INTER_NEAREST)  # NOUVEAU
                cv2.imshow(debug_names['total'], mask_total_disp)
                cv2.imshow(debug_names['orange'], mask_orange_disp)
                cv2.imshow(debug_names['jaune'], mask_jaune_disp)
                cv2.imshow(debug_names['blanc'], mask_blanc_disp)
                cv2.imshow(debug_names['beige'], mask_beige_disp)
                cv2.imshow(debug_names['jaune_vert'], mask_jaune_vert_disp)  # NOUVEAU
            else:
                for name in debug_names.values():
                    cv2.imshow(name, black)
            
            if success:
                cv2.putText(result, f"Cam: X={x_cam:+.1f} Y={y_cam:+.1f} Z={z_cam:+.1f} cm", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
                corners_int = corners.astype(np.int32)
                cv2.polylines(result, [corners_int], True, (0,255,0), 2)
                
                centre_couv = np.mean(corners_int, axis=0).astype(int)
                cv2.circle(result, tuple(centre_couv), 8, (255, 0, 255), -1)
                cv2.circle(result, tuple(centre_couv), 10, (255, 255, 255), 2)
                cv2.putText(result, "COVER CENTER", (centre_couv[0] + 15, centre_couv[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                cv2.line(result, (0, centre_couv[1]), (w, centre_couv[1]), (255, 0, 0), 2)
                
                diff_y = centre_couv[1] - centre_image_y
                couleur = (0, 255, 255) if diff_y > 0 else (255, 255, 0)
                cv2.putText(result, f"Diff Y: {diff_y:+d} px", (w-200, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, couleur, 1)
                
                # Calcul de l'angle
                hg = corners_int[0]
                hd = corners_int[1]
                dx = hd[0] - hg[0]
                dy = hd[1] - hg[1]
                angle_deg = math.degrees(math.atan2(dy, dx))
                cv2.putText(result, f"Angle couv: {angle_deg:+.1f} deg", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.line(result, tuple(hg), tuple(hd), (0, 255, 255), 2)
                
                labels = ["HG", "HD", "BD", "BG"]
                for i, (label, pt) in enumerate(zip(labels, corners_int)):
                    cv2.circle(result, tuple(pt), 4, (0, 0, 255), -1)
                    cv2.circle(result, tuple(pt), 6, (255, 255, 255), 2)
            else:
                cv2.putText(result, "Aucun couvercle detecte", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
            
            cv2.putText(result, "s: Sequence complete  d: Debug  q: Quitter", 
                        (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow("Detection + Asservissement", result)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):
                self.align_complet()
            elif key == ord('d'):
                self.debug = not self.debug
                status = "active" if self.debug else "desactive"
                self.get_logger().info(f"Debug {status}")
            elif key == ord('q'):
                break
            rclpy.spin_once(self, timeout_sec=0.0)
        
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Fini")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionAndControl()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrompu")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()