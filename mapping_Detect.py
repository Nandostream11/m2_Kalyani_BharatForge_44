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
import json
from std_msgs.msg import String  # Import String for publishing robot data

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
        
        # Subscribers for multiple robots
        self.robot_data = {}  # Dictionary to hold data for each robot

        for robot_id in range(1, 5):  # Assuming robot IDs are 1 to 4
            self.create_subscription(
                LaserScan, f'/robot{robot_id}/scan', self.create_scan_callback(robot_id), self.qos_profile
            )
            self.create_subscription(
                Odometry, f'/robot{robot_id}/odom', self.create_odom_callback(robot_id), self.qos_profile
            )
            self.create_subscription(
                Image, f'/robot{robot_id}/camera/image_raw', self.create_image_callback(robot_id), 10
            )
            self.create_subscription(
                CameraInfo, f'/robot{robot_id}/camera/camera_info', self.create_camera_info_callback(robot_id), 10
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
        
        # New Publisher for Robot Data
        self.robot_data_pub = self.create_publisher(String, 'robot_data', self.qos_profile)

        # Timers
        self.map_timer = self.create_timer(1.0, self.publish_maps)
        self.robot_data_timer = self.create_timer(1.0, self.publish_robot_data)  # Timer to publish robot data
        self.timer = self.create_timer(0.1, self.timer_callback)
        
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
        
        '''python'''
        map_data = []
        for _ in range(map_size):
            cell_value = -1 if random.random() < 0.9 else random.randint(50, 100)
            map_data.append(cell_value)
        
        map_msg.data = map_data
        
        return map_msg
    
    def create_scan_callback(self, robot_id):
        """Create a callback for LIDAR data for a specific robot"""
        def scan_callback(msg):
            self.robot_data.setdefault(robot_id, {})['latest_scan'] = msg
        return scan_callback
    
    def create_odom_callback(self, robot_id):
        """Create a callback for odometry data for a specific robot"""
        def odom_callback(msg):
            self.robot_data.setdefault(robot_id, {})['current_odom'] = msg
        return odom_callback
    
    def create_image_callback(self, robot_id):
        """Create a callback for camera data for a specific robot"""
        def image_callback(msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                results = self.model(frame)
                annotated_frame = results[0].plot()
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                self.detections_pub.publish(annotated_msg)
                cv2.imshow(f"YOLO Detection Robot {robot_id}", annotated_frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f'Error in image callback for robot {robot_id}: {e}')
        return image_callback
    
    def create_camera_info_callback(self, robot_id):
        """Create a callback for camera info data for a specific robot"""
        def camera_info_callback(msg):
            self.robot_data.setdefault(robot_id, {})['camera_info'] = msg
        return camera_info_callback
    
    def publish_maps(self):
        """Publish global and local maps"""
        now = self.get_clock().now().to_msg()
        
        global_map = self.global_map
        global_map.header.stamp = now
        self.global_map_pub.publish(global_map)
        
        # Local map publishing logic can be added here if needed
    
    def publish_robot_data(self):
        """Publish computed robot data for all robots"""
        for robot_id, data in self.robot_data.items():
            if 'latest_scan' in data and 'current_odom' in data:
                latest_scan = data['latest_scan']
                current_odom = data['current_odom']
                
                robot_data = {
                    "robot_id": f"R{robot_id}",
                    "location_x": current_odom.pose.pose.position.x,
                    "location_y": current_odom.pose.pose.position.y,
                    "obstacles_detected": self.check_obstacles(latest_scan),
                    "scan_ranges": latest_scan.ranges,
                    "camera_info": data.get('camera_info', None)  # Include camera info if available
                }
                self.robot_data_pub.publish(String(data=json.dumps(robot_data)))
                self.get_logger().info(f'Publishing robot data: {robot_data}')
    
    def check_obstacles(self, latest_scan):
        """Check for obstacles in the LiDAR scan"""
        if latest_scan is None:
            return True
        
        angle_min = latest_scan.angle_min
        angle_increment = latest_scan.angle_increment
        ranges = latest_scan.ranges
        
        min_angle = math.radians(-45)
        max_angle = math.radians(45)
        
        start_index = max(0, int((min_angle - angle_min) / angle_increment))
        end_index = min(len(ranges), int((max_angle - angle_min) / angle_increment))
        
        valid_ranges = [
            r for r in ranges[start_index:end_index] 
            if latest_scan.range_min < r < latest_scan.range_max 
            and not math.isinf(r) and not math.isnan(r)
        ]
        
        if not valid_ranges:
            return True
        
        min_distance = min(valid_ranges)
        
        return min_distance <= 1.0
    
    def timer_callback(self):
        """Main exploration logic callback"""
        if not all(robot_id in self.robot_data for robot_id in range(1, 5)):
            return  # Wait until we have data from all robots

        for robot_id in range(1, 5):
            data = self.robot_data[robot_id]
            if 'latest_scan' in data and 'current_odom' in data:
                latest_scan = data['latest_scan']
                current_odom = data['current_odom']

                twist = Twist()

                if self.check_obstacles(latest_scan):
                    twist.angular.z = 0.5  # Rotate if obstacles are detected
                    self.adjust_exploration_angle()  # Adjust the exploration angle
                else:
                    twist.linear.x = self.max_speed  # Move forward if no obstacles
                    
                    if random.random() < 0.1:
                        self.adjust_exploration_angle()  # Randomly adjust the exploration angle

                current_yaw = self.get_current_yaw(robot_id)
                angle_diff = self.angle_difference(current_yaw, self.target_angle)

                twist.angular.z += 0.5 * angle_diff  # Adjust the angular velocity based on the angle difference

                self.cmd_vel_pub.publish(twist)  # Publish the velocity command for the current robot

    def get_current_yaw(self, robot_id):
        """Extract current yaw from odometry for a specific robot"""
        if robot_id not in self.robot_data or 'current_odom' not in self.robot_data[robot_id]:
            return 0.0
        
        orientation = self.robot_data[robot_id]['current_odom'].pose.pose.orientation
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

    def adjust_exploration_angle(self):
        """Adjust the target exploration angle randomly"""
        self.target_angle += random.uniform(-math.pi / 4, math.pi / 4)  # Adjust by up to 45 degrees
        self.target_angle = self.target_angle % (2 * math.pi)  # Keep the angle within [0, 2π]

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