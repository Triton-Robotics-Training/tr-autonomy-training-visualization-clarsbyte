import rclpy
from rclpy.node import Node
from tr_messages.msg import SimGroundTruth
from tr_messages.msg import DetWithImg
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

class RobotBroadcaster(Node):
    def __init__(self):
        super().__init__("robot_broadcaster")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.detection_sub = self.create_subscription(
            DetWithImg,
            '/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        result = msg.detection_info.detections[0].results[0]
        pose = result.pose.pose

        self.get_logger().info(f"Panel detected at: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")

        self.broadcast_panel_transform(pose, msg.detection_info.header)

    def broadcast_panel_transform(self, pose, header):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "panel_3"

        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z

        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z
        transform.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = RobotBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()