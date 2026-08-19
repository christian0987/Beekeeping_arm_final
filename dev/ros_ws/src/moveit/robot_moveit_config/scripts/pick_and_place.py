#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        
        # 关节位置定义
        self.positions = {
            'home': [0.0, -3.14, 0.8, 0.0, 0.0, 0.0],
            'pre_grasp': [0.3, 0.0, 0.6, -0.05, -0.1, 0.001],
            'grasp': [0.3, 0.0, 0.1, -0.05, -0.1, 0.001],
            'lift': [0.3, 0.0, 0.4, -0.05, -0.1, 0.001],
            'pre_place': [0.0, 0.0, 0.6, -0.05, -0.1, 0.001],
            'place': [0.0, 0.0, 0.1, -0.05, -0.1, 0.001],
            'retract': [0.0, 0.0, 0.8, -0.05, -0.1, 0.001]
        }
        
        # 夹爪位置定义
        self.gripper_open = [-1.57, -1.57]  # 打开位置
        self.gripper_closed = [0.0, 0.0]   # 关闭位置
        
        # 关节名称列表 (根据URDF文件)
        self.arm_joint_names = [
            'robot_arm_base_turn',
            'robot_arm_turn_joint',
            'robot_arm_up_joint',
            'robot_arm_front_joint',
            'robot_arm_down_joint',
            'robot_hand_joint'
        ]
        
        self.gripper_joint_names = [
            'finger_1_joint',
            'finger_2_joint'
        ]
        
        # 创建动作客户端
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_group_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info("Pick and Place node initialized")

    def move_to_position(self, position_name, duration=5.0):
        """移动机械臂到指定位置"""
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        
        trajectory.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = self.positions[position_name]
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f"Moving to {position_name} position...")
        return self.send_goal(self.arm_client, goal_msg)

    def set_gripper_position(self, position, duration=2.0):
        """设置夹爪位置"""
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        
        trajectory.joint_names = self.gripper_joint_names
        point = JointTrajectoryPoint()
        point.positions = position
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        action = "opening" if position == self.gripper_open else "closing"
        self.get_logger().info(f"{action.capitalize()} gripper...")
        return self.send_goal(self.gripper_client, goal_msg)

    def send_goal(self, client, goal_msg):
        """发送目标并等待完成"""
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return False
            
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        if result.error_code != result.SUCCESSFUL:
            self.get_logger().error(f"Action failed with error code: {result.error_code}")
            return False
            
        return True

    def execute_pick_and_place(self):
        """执行完整的拾取和放置序列"""
        self.get_logger().info("Starting pick and place sequence")
        
        # 1. 回到HOME位置
        if not self.move_to_position('home'):
            self.get_logger().error("Failed to move to HOME position")
            return False
        
        # 2. 打开夹爪
        if not self.set_gripper_position(self.gripper_open):
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)
        
        # 3. 移动到预抓取位置
        if not self.move_to_position('pre_grasp'):
            self.get_logger().error("Failed to move to PRE_GRASP position")
            return False
        
        # 4. 下降到抓取位置
        if not self.move_to_position('grasp', duration=3.0):
            self.get_logger().error("Failed to move to GRASP position")
            return False
        time.sleep(0.5)
        
        # 5. 闭合夹爪（抓取箱子）
        if not self.set_gripper_position(self.gripper_closed):
            self.get_logger().error("Failed to close gripper")
            return False
        time.sleep(1.0)
        
        # 6. 抬起到安全高度
        if not self.move_to_position('lift'):
            self.get_logger().error("Failed to lift to safe height")
            return False
        
        # 7. 移动到放置预位置
        if not self.move_to_position('pre_place'):
            self.get_logger().error("Failed to move to PRE_PLACE position")
            return False
        
        # 8. 下降到放置位置
        if not self.move_to_position('place', duration=3.0):
            self.get_logger().error("Failed to move to PLACE position")
            return False
        time.sleep(0.5)
        
        # 9. 打开夹爪（释放箱子）
        if not self.set_gripper_position(self.gripper_open):
            self.get_logger().error("Failed to open gripper")
            return False
        time.sleep(1.0)
        
        # 10. 抬起到安全高度
        if not self.move_to_position('retract'):
            self.get_logger().error("Failed to retract to safe height")
            return False
        
        # 11. 回到home位置
        if not self.move_to_position('home'):
            self.get_logger().error("Failed to return to HOME position")
            return False
        
        self.get_logger().info("Pick and place sequence completed successfully")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    try:
        pick_and_place = PickAndPlace()
        
        # 执行拾取和放置序列
        success = pick_and_place.execute_pick_and_place()
        
        if not success:
            pick_and_place.get_logger().error("Pick and place sequence failed")
            sys.exit(1)
            
    except Exception as e:
        pick_and_place.get_logger().error(f"Error occurred: {str(e)}")
    finally:
        # 关闭节点
        pick_and_place.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
