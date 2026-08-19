#!/usr/bin/env python3
"""
Bridge entre MoveIt (trajectoire planifiée) et micro-ROS (robot réel)
Écoute la trajectoire planifiée par MoveIt et l'envoie au robot réel
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge')

        # Publishers pour le robot réel
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)

        # Subscriber à la trajectoire planifiée par MoveIt
        self.sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory',
            self.callback,
            10
        )

        self.get_logger().info('Trajectory Bridge demarre - Ecoute les trajectoires planifiees')

    def callback(self, msg):
        if not msg.points:
            return

        # Prend le dernier point de la trajectoire (position cible)
        target = msg.points[-1]
        arm_pos = [0.0, 0.0, 0.0]
        wrist_pos = [0.0, 0.0, 0.0]

        # Convertit les positions (assume ordre: 6 premiers joints)
        for i in range(min(3, len(target.positions))):
            arm_pos[i] = target.positions[i]
        for i in range(3, min(6, len(target.positions))):
            wrist_pos[i-3] = target.positions[i]

        # Publie vers le robot réel
        self.arm_pub.publish(Float64MultiArray(data=arm_pos))
        self.wrist_pub.publish(Float64MultiArray(data=wrist_pos))

        self.get_logger().info(f'Trajectoire planifiee - Arm: {arm_pos}, Wrist: {wrist_pos}')

def main(args=None):
    rclpy.init(args=args)
    bridge = TrajectoryBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
