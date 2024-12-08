#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json

class RobotSwarmCommNode(Node):
    def __init__(self):
        super().__init__('robot_swarm_comms')
        
        # QoS Profile Setup
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Declare the 'num_robots' parameter with a default value
        self.declare_parameter('num_robots', 4)  # Default value is 4
        
        # Get number of robots from parameters (passed via launch file)
        num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        
        # Robot Data Store
        self.robot_data = {}
        
        # Subscriptions for multiple robots (based on num_robots)
        for robot_id in range(1, num_robots + 1):
            self.create_subscription(
                LaserScan, f'/R{robot_id}/scan', self.create_scan_callback(robot_id), self.qos_profile
            )
            self.create_subscription(
                Odometry, f'/R{robot_id}/odom', self.create_odom_callback(robot_id), self.qos_profile
            )
            self.create_subscription(
                Image, f'/R{robot_id}/detected_objects', self.create_detected_objects_callback(robot_id), self.qos_profile
            )
            
        # Publisher for robot data topic
        self.robot_data_pub = self.create_publisher(String, 'robot_data', self.qos_profile)
        
        # Timer to publish robot data periodically
        self.robot_data_timer = self.create_timer(1.0, self.publish_robot_data)
        
        self.get_logger().info('Robot Swarm Comm Node Initialized')


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

    def create_detected_objects_callback(self, robot_id):
        """Create a callback for detected objects"""
        def detected_objects_callback(msg):
            self.robot_data.setdefault(robot_id, {})['detected_objects'] = {
            "objects": [{"cls": obj.cls, "conf": obj.conf, "bbox": obj.bbox} for obj in msg.objects]
        }
        return detected_objects_callback
    
    def publish_robot_data(self):
        for robot_id, data in self.robot_data.items():
            if 'latest_scan' in data and 'current_odom' in data:
                latest_scan = data['latest_scan']
                current_odom = data['current_odom']

                # Default values for obstacles and confidence
                obstacle_cls = None
                confidence_score = None

                # Check for detected objects and extract the most recent cls and conf
                if 'detected_objects' in data:
                    detected_objects_list = data['detected_objects'].get('objects', [])
                    obstacle_cls, confidence_score = self.extract_objects_info(detected_objects_list)

                # Construct the robot_data dictionary
                robot_data = {
                    "robot_id": f"R{robot_id}",
                    "location_x": current_odom.pose.pose.position.x,
                    "location_y": current_odom.pose.pose.position.y,
                    "obstacles": [obstacle_cls] if obstacle_cls else [],
                    "confidence_score": confidence_score if confidence_score else 0.0,
                    "scan_ranges": list(latest_scan.ranges),  # Convert to list for JSON serialization
                    "tasks_done": data.get('tasks_done', 0)  # Default to 0 if not present
                }

                try:
                    # Publish the data as a JSON string
                    json_str = json.dumps(robot_data)
                    self.robot_data_pub.publish(String(data=json_str))
                    self.get_logger().debug(f'Published robot data for R{robot_id}: {robot_data}')
                except Exception as e:
                    self.get_logger().error(f'Error publishing robot data: {e}')

    def check_obstacles(self, latest_scan):
        """Check for obstacles in the LiDAR scan"""
        if latest_scan is None:
            return False
        
        # Analyze scan ranges for obstacles
        min_obstacle_distance = 1.0  # Threshold distance to consider as an obstacle
        for range_value in latest_scan.ranges:
            if range_value < min_obstacle_distance:
                return True
        
        return False

    def extract_objects_info(self, detected_objects):
        """Extract the most recent class and confidence information from detected objects"""
        if not detected_objects:
            return None, None  # Default to None if no objects detected
        
        # Get the most confident detection from the list
        most_confident_detection = max(detected_objects, key=lambda obj: obj.conf)
        return (most_confident_detection.cls, most_confident_detection.conf)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSwarmCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
