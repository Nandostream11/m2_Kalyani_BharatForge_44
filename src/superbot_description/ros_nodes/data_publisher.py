#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        self.publisher = self.create_publisher(String, 'robot_data', 10)
        self.map_publisher = self.create_publisher(String, 'environment_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        robot_data = {
            "robot_id": "R1",               #{R1, R2, R3, R4}
            "location_x": 5, "location_y": 2,    #[(x,y): [(0.0),(10,10)]]
            "obstacles": ["wall"],          #["wall", "table", "human", ...]
            "confidence_score": 0.91,          #[0.6, 0.1, 0.2, ...,
            "tasks_done": 5                 #[0, 1, 2, 3, 4, 5]
        }
        self.publisher.publish(String(data=json.dumps(robot_data)))
        environment_data = {
            "environment_id": "E1",                             #Map ID
            "new_data": [{"x": 1, "y": 2, "type": "wall"}]      #[(x,y): [(0.0),(10,10)]]
        }
        self.map_publisher.publish(String(data=json.dumps(environment_data)))
        self.get_logger().info(f'Publishing: {robot_data}')

def main(args=None):
    rclpy.init(args=args)
    data_publisher = DataPublisher()
    rclpy.spin(data_publisher)
    data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()