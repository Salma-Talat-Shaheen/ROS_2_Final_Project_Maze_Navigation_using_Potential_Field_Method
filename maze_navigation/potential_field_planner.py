import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

# --- Simple Maze Planner (Completely isolated) ---
class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        # Declare parameters for goal coordinates
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 9.0)
        self.target_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('goal_y').get_parameter_value().double_value
        
        # Algorithm constants
        self.k_att = 1.2
        self.k_rep = 1.0
        self.d_obs = 1.5
        self.dist_threshold = 0.2
        self.v_max = 1.0
        
        # Robot state variables
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.scan_data, self.goal_reached, self.prev_w, self.chosen_side = None, False, 0.0, 0
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Timers
        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.print_status)
        self.get_logger().info(f'Navigator Started! Target Goal: ({self.target_x}, {self.target_y})')

    def odom_callback(self, msg):
        # Update current position and convert orientation to yaw
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg): self.scan_data = msg

    def print_status(self):
        # Display current distance to goal
        if not self.goal_reached:
            dist = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
            self.get_logger().info(f'Position: ({self.current_x:.2f}, {self.current_y:.2f}) | Dist: {dist:.2f}m')

    def control_loop(self):
        # Check if LiDAR data is available
        if self.scan_data is None: return
        dx, dy = self.target_x - self.current_x, self.target_y - self.current_y
        dist_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx) - self.current_yaw
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))
        
        # Goal check
        if dist_to_goal <= self.dist_threshold or self.goal_reached:
            self.stop_robot()
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(f'Goal Reached: Target ({self.target_x}, {self.target_y}) secured.')
            return
            
        # Process LiDAR ranges and handle invalid values
        ranges = np.array(self.scan_data.ranges)
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), 5.0, ranges)
        angles = np.linspace(self.scan_data.angle_min, self.scan_data.angle_max, len(ranges))
        path_mask = (angles > -0.3) & (angles < 0.3)
        min_path_dist = np.min(ranges[path_mask])
        
        msg = Twist()
        # Potential field logic: attractive vs repulsive
        if min_path_dist > self.d_obs:
            self.chosen_side = 0 
            target_w = self.k_att * angle_to_goal
            msg.linear.x = min(self.v_max, dist_to_goal)
        else:
            if self.chosen_side == 0: self.chosen_side = 1 if np.mean(ranges[angles > 0]) > np.mean(ranges[angles < 0]) else -1
            msg.linear.x = self.v_max * 0.3
            repulsion_strength = self.k_rep * (1.2 / max(min_path_dist, 0.4))
            target_w = (0.5 * angle_to_goal) + (self.chosen_side * repulsion_strength)
            
        # Apply smoothing to angular velocity
        msg.angular.z = (0.85 * self.prev_w) + (0.15 * target_w)
        self.prev_w = msg.angular.z
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self): self.cmd_vel_pub.publish(Twist())

# --- Complex Maze Planner (Completely isolated) ---
class SerpentineProPlanner(Node):
    def __init__(self):
        super().__init__('serpentine_pro_planner')
        self.target_x, self.target_y = 11.4, 11.4
        
        # Dynamic Parameters
        self.v_max = 0.95              # Maximum linear velocity for straight paths
        self.v_min = 0.35              # Minimum linear velocity for sharp turns
        self.k_att = 0.4               # Attractive gain towards the goal
        self.k_rep = 4.5               # Repulsive gain for obstacle avoidance
        self.k_mem = 3.0               # Memory gain to prevent backward movement
        
        # Internal states for memory and history
        self.path_history, self.last_pos, self.prev_w = [], (0.0, 0.0), 0.0
        self.current_x, self.current_y, self.current_yaw, self.scan_data = 0.0, 0.0, 0.0, None
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Pro Planner Started: Tracking Coordinates and High-Capacity Logic.')

    def odom_callback(self, msg):
        # Update pose and store path history
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        dist = math.sqrt((self.current_x - self.last_pos[0])**2 + (self.current_y - self.last_pos[1])**2)
        if dist > 0.25:
            self.path_history.append((self.current_x, self.current_y))
            self.last_pos = (self.current_x, self.current_y)
            if len(self.path_history) > 200: self.path_history.pop(0)

    def scan_callback(self, msg):
        # Clean LiDAR data
        ranges = np.array(msg.ranges)
        self.scan_data = np.where(np.isnan(ranges) | np.isinf(ranges), 5.0, ranges)
        self.scan_angles = np.linspace(msg.angle_min, msg.angle_max, len(self.scan_data))

    def control_loop(self):
        if self.scan_data is None: return
        dx, dy = self.target_x - self.current_x, self.target_y - self.current_y
        dist_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx) - self.current_yaw
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))
        
        if dist_to_goal < 0.3:
            self.get_logger().info('Goal Reached!')
            self.cmd_vel_pub.publish(Twist())
            return
            
        # Define scan sectors
        f_mask = (self.scan_angles > -0.2) & (self.scan_angles < 0.2)
        l_mask = (self.scan_angles >= 0.2) & (self.scan_angles < 0.9)
        r_mask = (self.scan_angles <= -0.2) & (self.scan_angles > -0.9)
        min_f = np.min(self.scan_data[f_mask]) if any(f_mask) else 5.0
        min_l = np.min(self.scan_data[l_mask]) if any(l_mask) else 5.0
        min_r = np.min(self.scan_data[r_mask]) if any(r_mask) else 5.0
        
        # Calculate memory force based on past trajectory
        mem_w = 0.0
        for px, py in self.path_history:
            mdist = math.sqrt((self.current_x - px)**2 + (self.current_y - py)**2)
            if 0.2 < mdist < 1.0:
                m_ang = math.atan2(self.current_y - py, self.current_x - px) - self.current_yaw
                mem_w += self.k_mem * math.atan2(math.sin(m_ang), math.cos(m_ang))
                
        # Motion planning logic
        msg = Twist()
        if min_f > 1.5:
            msg.linear.x = self.v_max
            msg.angular.z = (self.k_att * angle_to_goal) + (0.4 * mem_w)
        else:
            msg.linear.x = self.v_min
            side_factor = 1.0 if min_l > min_r else -1.0
            msg.angular.z = (side_factor * self.k_rep / max(min_f, 0.4)) + (0.5 * mem_w)
            
        # Smooth and publish output
        msg.angular.z = max(min(msg.angular.z, 1.8), -1.8)
        self.prev_w = (0.7 * self.prev_w) + (0.3 * msg.angular.z)
        msg.angular.z = self.prev_w
        self.cmd_vel_pub.publish(msg)
        
        # Debugging output
        print(f"Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | Yaw: {math.degrees(self.current_yaw):.1f}")
        print(f"Distances -> F: {min_f:.2f}, L: {min_l:.2f}, R: {min_r:.2f}")
        print(f"To Goal: {dist_to_goal:.2f} | Vel_X: {msg.linear.x:.2f}")
        print("-" * 50)

# --- Main system execution ---
def main(args=None):
    rclpy.init(args=args)
    # Select maze type via parameter
    temp_node = rclpy.create_node('temp_selector')
    temp_node.declare_parameter('maze_type', 'simple')
    maze = temp_node.get_parameter('maze_type').get_parameter_value().string_value
    temp_node.destroy_node()

    if maze == 'simple':
        node = PotentialFieldPlanner()
    else:
        node = SerpentineProPlanner()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
