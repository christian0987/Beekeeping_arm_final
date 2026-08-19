#!/usr/bin/env python3
#waypoint_executor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import sys

class WaypointExecutor(Node):
    """Publishes interpolated joint targets for arm and wrist, moving smoothly
       from current position to the next waypoint over the specified duration."""
    
    def __init__(self):
        super().__init__('waypoint_executor')
        
        # Publishers for arm and wrist
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)
        
        # Subscribers to get current joint positions (for smooth start)
        self.arm_state_sub = self.create_subscription(Float64MultiArray, 'arm/joint_state', self.arm_state_callback, 10)
        self.wrist_state_sub = self.create_subscription(Float64MultiArray, 'wrist/joint_state', self.wrist_state_callback, 10)
        
        # Store current positions (initialize to None, wait for first message)
        self.current_arm_pos = None   # [j0, j1, j2]
        self.current_wrist_pos = None # [j3, j4, j5]
        self.positions_received = False
        
        # Waypoint table (same as before)
        self.waypoints = [
            # arm_j0, arm_j1, arm_j2, wrist_j3, wrist_j4, wrist_j5, duration_sec
            
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0),
            #(0.0, 0.0, -220.0, 0.0, 0.0, 0.0, 12.0),
            #(0.0, 0.0, -220.0, 0.0, -50.0, 0.0, 4.0),
            
            #(200.0, 0.0, -220.0, 0.0, -50.0, 0.0, 4.0),          
            #(200.0, 0.0, -220.0, 0.0, -50.0, 0.0, 6.0),    
            #(200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.0),
                    
            #(0.0, 0.0, 0.0, 0.0, 0.0, 120.0, 6.0),
            #(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0),
        
            #(0.0, 0.0, 150.0, 0.0, 0.0, 0.0, 16.0),
            #(0.0, 180.0, 150.0, 0.0, 0.0, 0.0, 10.0),
            
            #(0.0, 180.0, 150.0, 0.0, -80.0, 0.0, 4.0),
            #(0.0, 180.0, 150.0, 0.0, -80.0, 120.0, 4.0),
            #(0.0, 180.0, 150.0, 0.0, -80.0, -90.0, 4.0),
            
            (0.0, 0.0, 0.0, 0.0, 0.0, 45, 16.0),            
            
        
            
            # (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0),
            # (0.0, 0.0, -250.0, 0.0, 0.0, 0.0, 20.0),
            # (0.0, 0.0, -250.0, 40.0, 0.0, 0.0, 4.0),
            # (0.0, 0.0, -250.0, 20.0, 0.0, 0.0, 4.0),
            # (0.0, 0.0, -250.0, 20.0, -50.0, 0.0, 4.0),
            
            # (200.0, 0.0, -250.0, 20.0, -50.0, 0.0, 4.0),
            # (200.0, 0.0, -250.0, 20.0, -50.0, 0.0, 4.0),
            
            # (200.0, 0.0, 0.0, 20.0, 0.0, 0.0, 12.0),
                    
            # (0.0, 0.0, 0.0, 0.0, 0.0, 180.0, 6.0),
            # (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0),
        
            # (0.0, 90.0, 150.0, 0.0, 0.0, 0.0, 10.0),
            # (0.0, 90.0, 150.0, 0.0, 0.0, 0.0, 6.0),
            
            # (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0),
            
            
            
            # (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0),
            # (200.0, 0.0, 0.0, 30.0, 0.0, 0.0, 5.0),
            # (-200.0, 0.0, 0.0, -30.0, 0.0, 0.0, 5.0),
            # (0.0, 90.0, 0.0, 0.0, 0.0, 0.0, 5.0),
            # (0.0, 120.0, 0.0, 0.0, 0.0, 0.0, 5.0),
            # (0.0, 0.0, 200.0, 0.0, 0.0, 0.0, 5.0),
            # (0.0, 0.0, -400.0, 0.0, 0.0, 0.0, 5.0),            
            # (0.0, 0.0, 0.0, 0.0, 80.0, 0.0, 5.0),
            # (0.0, 0.0, 0.0, 0.0, -80.0, 0.0, 5.0),
            # (0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 5.0),
            # (0.0, 0.0, 0.0, 0.0, 0.0, -90.0, 5.0),
            # (150.0, 45.0, 100.0, 40.0, 50.0, 45.0, 5.0),
            # (-150.0, 120.0, -100.0, -20.0, -40.0, -60.0, 5.0),
            # (300.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0),
            # (-300.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0),
            # (0.0, 120.0, 0.0, 0.0, 0.0, 0.0, 5.0),
            # (0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 5.0),
            # (0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 5.0),
            # (0.0, 0.0, 0.0, 0.0, 90.0, 0.0, 5.0),
            # (0.0, 0.0, 0.0, 0.0, 0.0, 170.0, 5.0),
            # (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0),
        ]
        
        self.publish_rate_hz = 50.0
        self.publish_period = 1.0 / self.publish_rate_hz
        
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        self.get_logger().info("Waiting for current joint positions from /arm/joint_state and /wrist/joint_state...")
    
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
            self.get_logger().info(f"Current positions received: arm={self.current_arm_pos}, wrist={self.current_wrist_pos}")
    
    def wait_for_initial_positions(self, timeout=5.0):
        """Wait up to timeout seconds for current positions, then use zeros as fallback."""
        start = time.time()
        while not self.positions_received and (time.time() - start) < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.positions_received:
            self.get_logger().warn("Timeout waiting for joint states. Using zero as initial positions.")
            self.current_arm_pos = [0.0, 0.0, 0.0]
            self.current_wrist_pos = [0.0, 0.0, 0.0]
            self.positions_received = True
    
    def run_sequence(self):
        """Execute all waypoints with linear interpolation."""
        # Wait for current positions to avoid jerky start
        self.wait_for_initial_positions()
        
        # Start from the actual current positions
        start_arm = self.current_arm_pos[:]
        start_wrist = self.current_wrist_pos[:]
        
        self.get_logger().info("Starting smooth waypoint execution...")
        
        for idx, wp in enumerate(self.waypoints):
            end_arm = [wp[0], wp[1], wp[2]]
            end_wrist = [wp[3], wp[4], wp[5]]
            duration = wp[6]
            
            self.get_logger().info(f"Waypoint {idx}: moving from arm={start_arm} to {end_arm}, "
                                   f"wrist from {start_wrist} to {end_wrist} in {duration}s")
            
            start_time = time.time()
            while time.time() - start_time < duration and rclpy.ok():
                elapsed = time.time() - start_time
                fraction = elapsed / duration  # goes from 0 to 1
                
                # Clamp fraction to [0,1] to avoid overshoot
                fraction = max(0.0, min(1.0, fraction))
                
                # Linear interpolation for each joint
                current_arm = [
                    start_arm[i] + (end_arm[i] - start_arm[i]) * fraction
                    for i in range(3)
                ]
                current_wrist = [
                    start_wrist[i] + (end_wrist[i] - start_wrist[i]) * fraction
                    for i in range(3)
                ]
                
                # Publish interpolated targets
                arm_msg = Float64MultiArray()
                arm_msg.data = current_arm
                self.arm_pub.publish(arm_msg)
                
                wrist_msg = Float64MultiArray()
                wrist_msg.data = current_wrist
                self.wrist_pub.publish(wrist_msg)
                
                # Log once per waypoint (optional)
                if fraction < 0.05 and elapsed < 0.1:
                    self.get_logger().debug(f"Started interpolation to waypoint {idx}")
                
                # Maintain publishing rate
                time.sleep(self.publish_period)
                rclpy.spin_once(self, timeout_sec=0.0)
            
            # Ensure final target is reached exactly
            arm_msg = Float64MultiArray()
            arm_msg.data = end_arm
            self.arm_pub.publish(arm_msg)
            
            wrist_msg = Float64MultiArray()
            wrist_msg.data = end_wrist
            self.wrist_pub.publish(wrist_msg)
            
            # Update start positions for next waypoint
            start_arm = end_arm[:]
            start_wrist = end_wrist[:]
            
            # Check for shutdown
            if not rclpy.ok():
                break
        
        self.get_logger().info("Waypoint sequence completed.")
    
    def load_from_csv(self, csv_path):
        """Optional: load waypoints from CSV file."""
        import csv
        self.waypoints = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 7:
                    self.waypoints.append(tuple(float(x) for x in row[:7]))
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {csv_path}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointExecutor()
    
    # Uncomment to load from CSV instead of hardcoded table
    # node.load_from_csv("waypoints.csv")
    
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()