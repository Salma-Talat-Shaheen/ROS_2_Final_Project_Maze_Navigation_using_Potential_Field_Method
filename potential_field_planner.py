#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

class HybridMazePlanner(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('hybrid_planner')
        
        # --- Parameters (To support CLI execution) ---
        self.declare_parameter('goal_x', 9.0) # Declare goal_x parameter
        self.declare_parameter('goal_y', 9.0) # Declare goal_y parameter
        self.target_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('goal_y').get_parameter_value().double_value
        
        # --- State Variables ---
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0 # Robot pose
        self.scan_data = None # Store LiDAR scan data
        self.goal_reached = False # Goal status flag
        self.path_history = [] # History for complex maze memory
        self.last_pos = (0.0, 0.0) # Last recorded position for history
        self.prev_w = 0.0 # Previous angular velocity for smoothing
        self.chosen_side = 0 # Chosen avoidance side for simple maze
        
        # --- ROS Communication ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher for velocity
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10) # Subscriber for LiDAR
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # Subscriber for Odometry
        self.create_timer(0.05, self.control_loop) # Control loop timer (20Hz)
        
        # Unified Print Statement
        self.get_logger().info(f'Planner Initialized potinal file. Goal: ({self.target_x}, {self.target_y})')

    def odom_callback(self, msg):
        # Update robot pose from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw angle
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        
        # Track path for Complex Maze spatial memory (record every 25cm)
        dist = math.sqrt((self.current_x - self.last_pos[0])**2 + (self.current_y - self.last_pos[1])**2)
        if dist > 0.25:
            self.path_history.append((self.current_x, self.current_y))
            self.last_pos = (self.current_x, self.current_y)
            # Keep history size limited
            if len(self.path_history) > 200: self.path_history.pop(0)

    def scan_callback(self, msg):
        # Update scan data
        self.scan_data = msg

    def control_loop(self):
        # Exit if no scan data or goal reached
        if self.scan_data is None or self.goal_reached: return

        # Switch logic: Determine mode based on goal coordinates
        is_complex = self.target_x > 10.0
        
        # Preprocess scan data (remove invalid readings)
        ranges = np.where(np.isnan(self.scan_data.ranges) | np.isinf(self.scan_data.ranges), 5.0, np.array(self.scan_data.ranges))
        angles = np.linspace(self.scan_data.angle_min, self.scan_data.angle_max, len(ranges))
        msg = Twist()

        # --- MODE 1: Simple Maze (Potential Field Logic) ---
        if not is_complex:
            # Parameters Configuration:
            k_att = 1.2    # Attractive gain towards the goal
            k_rep = 1.0    # Repulsive gain from obstacles
            d_obs = 1.5    # Obstacle influence distance threshold
            v_max = 1.0    # Maximum linear velocity
            
            # Calculate distance and angle to goal
            dx, dy = self.target_x-self.current_x, self.target_y-self.current_y
            dist_to_goal = math.sqrt(dx**2 + dy**2)
            angle = math.atan2(math.sin(math.atan2(dy, dx)-self.current_yaw), math.cos(math.atan2(dy, dx)-self.current_yaw))
            
            # Check for goal arrival
            if dist_to_goal <= 0.2: self.goal_reached = True; return
            
            # Detect obstacles in path
            min_dist = np.min(ranges[(angles > -0.3) & (angles < 0.3)])
            
            if min_dist > d_obs:
                # Clear path logic
                self.chosen_side = 0
                target_w = k_att * angle
                msg.linear.x = min(v_max, dist_to_goal)
            else:
                # Obstacle detected: decide side and apply repulsion
                if self.chosen_side == 0: self.chosen_side = 1 if np.mean(ranges[angles > 0]) > np.mean(ranges[angles < 0]) else -1
                msg.linear.x = v_max * 0.3
                target_w = (0.5 * angle) + (self.chosen_side * k_rep * (1.2 / max(min_dist, 0.4)))
            
            # Smooth angular velocity
            self.prev_w = (0.85 * self.prev_w) + (0.15 * target_w)
            msg.angular.z = self.prev_w

        # --- MODE 2: Complex Maze (Pro Planner Logic) ---
        else:
            # Parameters Configuration:
            v_max = 0.95   # Max velocity for straight paths
            v_min = 0.35   # Min velocity for tight turns
            k_att = 0.4    # Attraction gain to goal
            k_rep = 4.5    # Repulsive gain from obstacles
            k_mem = 3.0    # Memory gain to avoid re-entering traps
            
            # Calculate goal vector
            dx, dy = self.complex_target[0]-self.current_x, self.complex_target[1]-self.current_y
            angle = math.atan2(math.sin(math.atan2(dy, dx)-self.current_yaw), math.cos(math.atan2(dy, dx)-self.current_yaw))
            
            # Divide scan into sectors (front, left, right)
            f_mask = (angles > -0.2) & (angles < 0.2)
            l_mask = (angles >= 0.2) & (angles < 0.9)
            r_mask = (angles <= -0.2) & (angles > -0.9)
            min_f, min_l, min_r = np.min(ranges[f_mask]), np.min(ranges[l_mask]), np.min(ranges[r_mask])
            
            # Calculate memory force based on past path
            mem_w = 0.0
            for px, py in self.path_history:
                mdist = math.sqrt((self.current_x-px)**2 + (self.current_y-py)**2)
                if 0.2 < mdist < 1.0:
                    m_ang = math.atan2(self.current_y-py, self.current_x-px) - self.current_yaw
                    mem_w += k_mem * math.atan2(math.sin(m_ang), math.cos(m_ang))

            # Steering logic for complex maze
            if min_f > 1.5:
                # Path is open
                msg.linear.x = v_max
                msg.angular.z = (k_att * angle) + (0.4 * mem_w)
            else:
                # Obstacle encountered: steer away
                msg.linear.x = v_min
                side = 1.0 if min_l > min_r else -1.0
                msg.angular.z = (side * k_rep / max(min_f, 0.4)) + (0.5 * mem_w)
            
            # Constrain and smooth angular velocity
            msg.angular.z = max(min(msg.angular.z, 1.8), -1.8)
            self.prev_w = (0.7 * self.prev_w) + (0.3 * msg.angular.z)
            msg.angular.z = self.prev_w

        # Publish velocity command
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    node = HybridMazePlanner()
    # Spin the node
    rclpy.spin(node)
    # Cleanup on exit
    rclpy.shutdown()

if __name__ == '__main__':
    main()