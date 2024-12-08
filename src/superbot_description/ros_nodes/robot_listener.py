#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
import json
import sys
import os
import math
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from database.data_logging import log_robot_data, update_dynamic_map
from database.mongo_setup import connect_to_db

class RobotListener(Node):
    def __init__(self):
        super().__init__('robot_listener')

        # QoS Profile Setup
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        # Subscriber for robot_data
        self.subscription = self.create_subscription(
            String,
            'robot_data',
            self.listener_callback,
            self.qos_profile
        )
        
        # Publisher for environment_data
        self.environment_data_pub = self.create_publisher(
            String,
            'environment_data',
            self.qos_profile
        )
        self.db = connect_to_db()
        self.get_logger().info("Robot listener node started")

    def listener_callback(self, msg):
        # Parse robot_data message
        robot_data = json.loads(msg.data)
        robot_x = robot_data['location_x']
        robot_y = robot_data['location_y']
        scan_ranges = robot_data['scan_ranges']
        start_angle = robot_data['start_angle']  # Assume start angle is provided
        angle_increment = robot_data['angle_increment']  # Assume angle increment is provided
        
        # Calculate obstacle coordinates
        obstacle_coordinates = self.calculate_obstacle_coordinates(scan_ranges, robot_x, robot_y, start_angle, angle_increment)
        
        # Log robot data to the database
        log_robot_data(
            self.db,
            robot_data['robot_id'],
            robot_data['location_x'],
            robot_data['location_y'],
            robot_data['obstacles'],
            robot_data['confidence_score'],
            robot_data['scan_ranges'],
            robot_data['tasks_done']
        )
        self.get_logger().info(f"Logged robot data: {robot_data}")

        # Prepare environment_data message
        environment_data = {
            "robot_id": robot_data['robot_id'],
            "location_x": robot_data['location_x'],
            "location_y": robot_data['location_y'],
            "obstacles": robot_data['obstacles']
        }

        # Publish to environment_data topic and update dynamic map
        self.publish_environment_data(environment_data, obstacle_coordinates)

    def publish_environment_data(self, environment_data, obstacle_coordinates):
        try:
            # Convert the environment_data to JSON string and publish
            json_str = json.dumps(environment_data)
            self.environment_data_pub.publish(String(data=json_str))
            self.get_logger().info(f"Published environment data: {environment_data}")

            # Run map_listener_callback functionality to update the map
            self.update_map(environment_data, obstacle_coordinates)
        except Exception as e:
            self.get_logger().error(f"Error publishing environment data: {e}")

    def update_map(self, environment_data, obstacle_coordinates):
        try:
            # Update the map with obstacle coordinates as separate location_x, location_y
            for i, obstacle in enumerate(obstacle_coordinates):
                map_data = {
                    "local_id": environment_data['robot_id'],
                    "new_data": {
                        "obs_x": obstacle['x'],  # Obstacle's x-coordinate
                        "obs_y": obstacle['y'],  # Obstacle's y-coordinate
                        "obstacles": environment_data['obstacles']  # Obstacle recognition
                    }
                }
                update_dynamic_map(self.db, map_data['local_id'], map_data['new_data'])
                self.get_logger().info(f"Updated dynamic map: {map_data}")
        except Exception as e:
            self.get_logger().error(f"Error updating dynamic map: {e}")

    def calculate_obstacle_coordinates(self, scan_ranges, robot_x, robot_y, start_angle, angle_increment):
        obstacle_coordinates = []
        
        for i, range_value in enumerate(scan_ranges):
            if range_value > 0.0:  # Ignore invalid or zero range values
                theta_i = start_angle + i * angle_increment  # Angle for this scan
                obstacle_x = robot_x + range_value * math.cos(theta_i)
                obstacle_y = robot_y + range_value * math.sin(theta_i)
                obstacle_coordinates.append({"x": obstacle_x, "y": obstacle_y})
        
        return obstacle_coordinates

def main(args=None):
    rclpy.init(args=args)
    robot_listener = RobotListener()
    rclpy.spin(robot_listener)
    robot_listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
