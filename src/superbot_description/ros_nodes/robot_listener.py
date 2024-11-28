import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from database.data_logging import log_robot_data
from database.mongo_setup import connect_to_db

# from database.data_logging import log_robot_data
# from database.mongo_setup import connect_to_db

class RobotListener(Node):
    def __init__(self):
        super().__init__('robot_listener')
        self.subscription = self.create_subscription(
            String,
            'robot_data',
            self.listener_callback,
            10
        )
        self.db = connect_to_db()
        print("Robot listener node started")

    def listener_callback(self, msg):
        robot_data = json.loads(msg.data)
        log_robot_data(self.db, robot_data['robot_id'], robot_data['location'], robot_data['obstacles'], robot_data['tasks_completed'])
        print("Logged robot data")

def main(args=None):
    rclpy.init(args=args)
    robot_listener = RobotListener()
    rclpy.spin(robot_listener)
    robot_listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
