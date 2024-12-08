#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
import random
import cv2
import torch

class MappingAndDetectionNode(Node):
    def __init__(self):
        super().__init__('mapping_detection_node')
        
        # QoS Profile Setup
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
                ('grid_size', 10.0),     # meters
                ('resolution', 0.05),    # meters per cell
                ('max_speed', 0.5),      # m/s
                ('min_obstacle_distance', 0.25)  # meters
            ]
        )
        
        # Get parameters
        self.grid_size = self.get_parameter('grid_size').value
        self.resolution = self.get_parameter('resolution').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        
        # YOLO Object Detection Setup
        self.model = YOLO('yolov8n.pt')  # Load YOLO model
        self.bridge = CvBridge()
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, self.qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_cont/odom', self.odom_callback, self.qos_profile
        )
        
        self.local_map_sub = self.create_subscription(
            OccupancyGrid, '/local_map', self.local_map_callback, self.qos_profile
        )
        
        # Camera Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', self.qos_profile
        )
        
        self.global_map_pub = self.create_publisher(
            OccupancyGrid, '/global_map', self.qos_profile
        )
        
        self.local_map_pub = self.create_publisher(
            OccupancyGrid, '/local_map', self.qos_profile
        )
        
        # Detections Publisher (Optional)
        self.detections_pub = self.create_publisher(
            Image, '/detected_objects', self.qos_profile
        )
        
        # Timers
        self.map_timer = self.create_timer(1.0, self.publish_maps)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State variables
        self.latest_scan = None
        self.current_odom = None
        self.local_map = None
        self.latest_image = None
        
        # Exploration state
        self.target_angle = 0.0
        
        # Initial map generation
        self.global_map = self.generate_map()
        
        self.get_logger().info('Mapping and Detection Node Initialized')
    
    def generate_map(self):
        """Generate a sample occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.resolution
        map_msg.info.width = int(self.grid_size / self.resolution)
        map_msg.info.height = int(self.grid_size / self.resolution)
        
        map_msg.info.origin.position.x = -self.grid_size / 2
        map_msg.info.origin.position.y = -self.grid_size / 2
        map_msg.info.origin.position.z = 0.0
        
        map_size = map_msg.info.width * map_msg.info.height
        
        map_data = []
        for _ in range(map_size):
            cell_value = -1 if random.random() < 0.9 else random.randint(50, 100)
            map_data.append(cell_value)
        
        map_msg.data = map_data
        
        return map_msg
    
    def publish_maps(self):
        """Publish global and local maps"""
        now = self.get_clock().now().to_msg()
        
        global_map = self.global_map
        global_map.header.stamp = now
        self.global_map_pub.publish(global_map)
        
        if self.local_map is None:
            local_map = self.generate_map()
            local_map.header.stamp = now
            local_map.header.frame_id = 'odom'
            self.local_map_pub.publish(local_map)
        else:
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
    
    def image_callback(self, msg):
        """Process incoming camera image with YOLO detection"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLO Inference
            results = self.model(frame)
            
            # Draw results on the frame
            annotated_frame = results[0].plot()
            
            # Convert annotated frame back to ROS message
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            
            # Publish annotated image
            self.detections_pub.publish(annotated_msg)
            
            # Store latest image for potential future use
            self.latest_image = frame

            cv2.imshow("YOLO Detection", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')
    
    def check_obstacles(self):
        """Check for obstacles in the LiDAR scan"""
        if self.latest_scan is None:
            return True
        
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges
        
        min_angle = math.radians(-45)
        max_angle = math.radians(45)
        
        start_index = max(0, int((min_angle - angle_min) / angle_increment))
        end_index = min(len(ranges), int((max_angle - angle_min) / angle_increment))
        
        valid_ranges = [
            r for r in ranges[start_index:end_index] 
            if self.latest_scan.range_min < r < self.latest_scan.range_max 
            and not math.isinf(r) and not math.isnan(r)
        ]
        
        if not valid_ranges:
            return True
        
        min_distance = min(valid_ranges)
        
        if min_distance <= 1.0:
            return True
        
        return False
    
    def adjust_exploration_angle(self):
        """Increment target angle"""
        self.target_angle += math.radians(1)
        self.target_angle = (self.target_angle + math.pi) % (2 * math.pi) - math.pi
    
    def timer_callback(self):
        """Main exploration logic callback"""
        if (self.latest_scan is None or 
            self.current_odom is None or 
            self.local_map is None):
            return
        
        twist = Twist()
        
        if self.check_obstacles():
            twist.angular.z = 0.5
            self.adjust_exploration_angle()
        else:
            twist.linear.x = self.max_speed
            
            if random.random() < 0.1:
                self.adjust_exploration_angle()
        
        current_yaw = self.get_current_yaw()
        angle_diff = self.angle_difference(current_yaw, self.target_angle)
        
        twist.angular.z += 0.5 * angle_diff
        
        self.cmd_vel_pub.publish(twist)
    
    def get_current_yaw(self):
        """Extract current yaw from odometry"""
        if self.current_odom is None:
            return 0.0
        
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
    node = MappingAndDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()