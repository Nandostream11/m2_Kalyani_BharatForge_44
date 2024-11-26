import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class WheelTransformPublisher(Node):
    def __init__(self):
        super().__init__('wheel_transform_publisher')
        self.broadcaster = TransformBroadcaster(self)

    def publish_wheel_transforms(self):
        # Create a TransformStamped message for left and right wheels
        left_wheel_transform = TransformStamped()
        left_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        left_wheel_transform.header.frame_id = 'base_link'
        left_wheel_transform.child_frame_id = 'left_wheel'
        left_wheel_transform.transform.translation.x = 0.5
        left_wheel_transform.transform.translation.y = 0.0
        left_wheel_transform.transform.translation.z = 0.1
        left_wheel_transform.transform.rotation.x = 0.0
        left_wheel_transform.transform.rotation.y = 0.0
        left_wheel_transform.transform.rotation.z = 0.0
        left_wheel_transform.transform.rotation.w = 1.0

        right_wheel_transform = TransformStamped()
        right_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        right_wheel_transform.header.frame_id = 'base_link'
        right_wheel_transform.child_frame_id = 'right_wheel'
        right_wheel_transform.transform.translation.x = -0.5
        right_wheel_transform.transform.translation.y = 0.0
        right_wheel_transform.transform.translation.z = 0.1
        right_wheel_transform.transform.rotation.x = 0.0
        right_wheel_transform.transform.rotation.y = 0.0
        right_wheel_transform.transform.rotation.z = 0.0
        right_wheel_transform.transform.rotation.w = 1.0

        # Broadcast the transforms
        self.broadcaster.sendTransform(left_wheel_transform)
        self.broadcaster.sendTransform(right_wheel_transform)

def main(args=None):
    rclpy.init(args=args)
    node = WheelTransformPublisher()

    while rclpy.ok():
        node.publish_wheel_transforms()
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
