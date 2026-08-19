#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveItBridge(Node):
    """
    Bridge entre MoveIt et micro-ROS
    - Convertit /joint_states (MoveIt) → /arm/joint_state et /wrist/joint_state
    - Convertit /joint_trajectory_controller/follow_joint_trajectory → /arm/joint_position et /wrist/joint_position
    """
    
    def __init__(self):
        super().__init__('moveit_bridge')
        
        # Mapping des noms de joints MoveIt vers les index
        # À ADAPTER selon les noms exacts de tes joints
        self.joint_mapping = {
            'robot_arm_base_turn': 0,    # joint 0
            'robot_arm_down_joint': 1,    # joint 1
            'robot_arm_front_joint': 2,   # joint 2
            'robot_arm_up_joint': 3,      # joint 3 (wrist)
            'robot_arm_turn_joint': 4,    # joint 4 (wrist)
            'finger_1_joint': 5,          # joint 5 (wrist)
            'finger_2_joint': 6,
        }
        
        # Publishers pour micro-ROS
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)
        self.arm_state_pub = self.create_publisher(Float64MultiArray, 'arm/joint_state', 10)
        self.wrist_state_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_state', 10)
        
        # Subscribers pour MoveIt
        self.joint_state_sub = self.create_subscription(
            JointState, 
            'joint_states', 
            self.joint_state_callback, 
            10
        )
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        self.get_logger().info("MoveIt Bridge démarré")
    
    def joint_state_callback(self, msg):
        """Convertit /joint_states → /arm/joint_state et /wrist/joint_state"""
        arm_positions = [0.0, 0.0, 0.0]
        wrist_positions = [0.0, 0.0, 0.0]
        
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                if name in self.joint_mapping:
                    idx = self.joint_mapping[name]
                    if idx < 3:
                        arm_positions[idx] = msg.position[i]
                    else:
                        wrist_positions[idx - 3] = msg.position[i]
        
        # Publier sur micro-ROS
        arm_msg = Float64MultiArray()
        arm_msg.data = arm_positions
        self.arm_state_pub.publish(arm_msg)
        
        wrist_msg = Float64MultiArray()
        wrist_msg.data = wrist_positions
        self.wrist_state_pub.publish(wrist_msg)
    
    def trajectory_callback(self, msg):
        """Convertit trajectoire MoveIt → commandes micro-ROS"""
        if not msg.points:
            return
        
        # Dernier point de la trajectoire (position cible)
        target = msg.points[-1]
        arm_positions = [0.0, 0.0, 0.0]
        wrist_positions = [0.0, 0.0, 0.0]
        
        for i, name in enumerate(msg.joint_names):
            if i < len(target.positions):
                if name in self.joint_mapping:
                    idx = self.joint_mapping[name]
                    if idx < 3:
                        arm_positions[idx] = target.positions[i]
                    else:
                        wrist_positions[idx - 3] = target.positions[i]
        
        # Publier sur micro-ROS
        arm_msg = Float64MultiArray()
        arm_msg.data = arm_positions
        self.arm_pub.publish(arm_msg)
        
        wrist_msg = Float64MultiArray()
        wrist_msg.data = wrist_positions
        self.wrist_pub.publish(wrist_msg)
        
        self.get_logger().info(f"Trajectoire envoyée: arm={arm_positions}, wrist={wrist_positions}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
