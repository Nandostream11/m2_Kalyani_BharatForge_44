import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        self.publisher = self.create_publisher(String, 'robot_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        robot_data = {
            "robot_id": "robot_1",
            "location": "Room A",
            "obstacles": ["wall", "table"],
            "tasks_completed": 5
        }
        self.publisher.publish(String(data=json.dumps(robot_data)))
        self.get_logger().info(f'Publishing: {robot_data}')

def main(args=None):
    rclpy.init(args=args)
    data_publisher = DataPublisher()
    rclpy.spin(data_publisher)
    data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()