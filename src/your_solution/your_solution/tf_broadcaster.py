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

        self.camera_sub = self.create_subscription(
            SimGroundTruth,
            'simulation/ground_truth',
            self.camera_callback,
            10
        )

    def detection_callback(self, msg):
        result = msg.detection_info.detections[0].results[0]
        pose = result.pose.pose

        self.get_logger().info(f"Panel detected at: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")

        self.broadcast_panel_transform(pose, msg.detection_info.header)
    
    def camera_callback(self, msg):
        pose = msg.primary_robot.camera_pose

        self.get_logger().info(f"Camera at: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
       
        self.broadcast_camera_transform(pose)

    def broadcast_panel_transform(self, pose, header):
        p3 = TransformStamped()
        

        p3.header.stamp = self.get_clock().now().to_msg()
        p3.header.frame_id = "map"
        p3.child_frame_id = "panel_3"

        p3.transform.translation.x = pose.position.x
        p3.transform.translation.y = pose.position.y
        p3.transform.translation.z = pose.position.z

        p3.transform.rotation.x = pose.orientation.x
        p3.transform.rotation.y = pose.orientation.y
        p3.transform.rotation.z = pose.orientation.z
        p3.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(p3)
    
    def broadcast_camera_transform(self, c_pose):
        c = TransformStamped() #c_pose = msg.primary_robot.camera_pose

        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = "map"
        c.child_frame_id = "camera_frame"

        c.transform.translation.x = c_pose.position.x
        c.transform.translation.y = c_pose.position.y
        c.transform.translation.z = c_pose.position.z

        c.transform.rotation.x = c_pose.orientation.x
        c.transform.rotation.y = c_pose.orientation.y
        c.transform.rotation.z = c_pose.orientation.z
        c.transform.rotation.w = c_pose.orientation.w

        self.tf_broadcaster.sendTransform(c)


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