import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np
import random

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('dynamic_mapper')
        
        # Create a more explicit QoS profile with RELIABLE reliability
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Node Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('grid_size', 10.0),  # meters
                ('resolution', 0.05),  # meters per cell
                ('max_speed', 0.5),    # m/s
                ('min_obstacle_distance', 0.25)  # meters
            ]
        )
        
        # Get parameters
        self.grid_size = self.get_parameter('grid_size').value
        self.resolution = self.get_parameter('resolution').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            self.qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/diff_cont/odom', 
            self.odom_callback, 
            self.qos_profile
        )
        
        self.local_map_sub = self.create_subscription(
            OccupancyGrid, 
            '/local_map', 
            self.local_map_callback, 
            self.qos_profile
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/diff_cont/cmd_vel_unstamped', 
            self.qos_profile
        )
        
        # Map Publishers
        self.global_map_pub = self.create_publisher(
            OccupancyGrid, 
            '/global_map', 
            self.qos_profile
        )
        
        self.local_map_pub = self.create_publisher(
            OccupancyGrid, 
            '/local_map', 
            self.qos_profile
        )
        
        # Timer for map publishing (every 1 second)
        self.map_timer = self.create_timer(1.0, self.publish_maps)
        
        # Timer for periodic updates (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State variables
        self.latest_scan = None
        self.current_odom = None
        self.local_map = None
        
        # Exploration angle
        self.target_angle = 0.0
        
        # Map generation
        self.global_map = self.generate_map()
        
        self.get_logger().info('Exploration Node Initialized')
    
    def generate_map(self):
        """Generate a sample occupancy grid map"""
        map_msg = OccupancyGrid()
        
        # Set map metadata
        map_msg.header.frame_id = 'map'
        
        # Map dimensions
        map_msg.info.resolution = self.resolution
        map_msg.info.width = int(self.grid_size / self.resolution)
        map_msg.info.height = int(self.grid_size / self.resolution)
        
        # Origin position (center of the map)
        map_msg.info.origin.position.x = -self.grid_size / 2
        map_msg.info.origin.position.y = -self.grid_size / 2
        map_msg.info.origin.position.z = 0.0
        
        # Generate map data
        map_size = map_msg.info.width * map_msg.info.height
        
        # Create a more realistic map with some obstacles
        map_data = []
        for _ in range(map_size):
            # 10% chance of obstacle, 90% free space
            cell_value = -1 if random.random() < 0.9 else random.randint(50, 100)
            map_data.append(cell_value)
        
        map_msg.data = map_data
        
        return map_msg
    
    def publish_maps(self):
        """Publish global and local maps"""
        # Update timestamp
        now = self.get_clock().now().to_msg()
        
        # Global Map
        global_map = self.global_map
        global_map.header.stamp = now
        self.global_map_pub.publish(global_map)
        
        # Local Map (if available, otherwise generate)
        if self.local_map is None:
            local_map = self.generate_map()
            local_map.header.stamp = now
            local_map.header.frame_id = 'odom'
            self.local_map_pub.publish(local_map)
        else:
            # Update timestamp of received local map
            self.local_map.header.stamp = now
            self.local_map_pub.publish(self.local_map)
    
    def scan_callback(self, msg):
        """Process incoming LiDAR scan data"""
        self.latest_scan = msg
    
    def odom_callback(self, msg):
        """Update current robot odometry"""
        self.current_odom = msg
    
    def local_map_callback(self, msg):
        """Update local map information"""
        self.local_map = msg

    
    def check_obstacles(self):
        """Check for obstacles in the LiDAR scan"""
        if self.latest_scan is None:
            return True  # Assume obstacles if no scan data
        
        # Find minimum distance in scan
        ranges = self.latest_scan.ranges
        try:
            min_distance = min(
                [r for r in ranges if r > self.latest_scan.range_min and r < self.latest_scan.range_max]
            )
            return min_distance < self.min_obstacle_distance
        except ValueError:
            # If no valid ranges are found
            return True
    
    def adjust_exploration_angle(self):
        """Adjust target angle for exploration"""
        self.target_angle = random.uniform(-math.pi, math.pi)
    
    def timer_callback(self):
        """Main exploration logic callback"""
        # Check if all required data is available
        if (self.latest_scan is None or 
            self.current_odom is None or 
            self.local_map is None):
            return
        
        # Create velocity command
        twist = Twist()
        
        # Obstacle detection
        if self.check_obstacles():
            # Turn in place if obstacle is too close
            twist.angular.z = 0.5  # Rotate at 0.5 rad/s
            
            # Adjust exploration angle when turning
            self.adjust_exploration_angle()
        else:
            # Move forward
            twist.linear.x = self.max_speed
            
            # Periodically adjust exploration angle
            if random.random() < 0.1:  # 10% chance each cycle
                self.adjust_exploration_angle()
        
        # Apply exploration angle adjustment
        current_yaw = self.get_current_yaw()
        angle_diff = self.angle_difference(current_yaw, self.target_angle)
        
        # Add angular velocity to approach target angle
        twist.angular.z += 0.5 * angle_diff
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
    
    def get_current_yaw(self):
        """Extract current yaw from odometry"""
        if self.current_odom is None:
            return 0.0
        
        # Extract yaw from quaternion
        orientation = self.current_odom.pose.pose.orientation
        return self.quaternion_to_yaw(orientation)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def angle_difference(self, angle1, angle2):
        """Calculate the smallest angle between two angles"""
        diff = angle2 - angle1
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return diff

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()