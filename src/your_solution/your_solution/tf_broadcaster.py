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
        c_pose = msg.primary_robot.camera_pose
        panels_pose = msg.secondary_robot.armor_panel_poses

        self.broadcast_camera_panel_transform(c_pose, panels_pose)

    def broadcast_panel_transform(self, pose, header):
        d = TransformStamped()
        
        d.header.stamp = self.get_clock().now().to_msg()
        d.header.frame_id = "camera_frame"
        d.child_frame_id = "detected_panel"

        d.transform.translation.x = pose.position.x
        d.transform.translation.y = pose.position.y
        d.transform.translation.z = pose.position.z

        d.transform.rotation.x = pose.orientation.x
        d.transform.rotation.y = pose.orientation.y
        d.transform.rotation.z = pose.orientation.z
        d.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(d)
    
    def panel_transform(self, panels_pose, i):
        p = TransformStamped()

        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "map"
        p.child_frame_id = f"panel_{i}"

        p.transform.translation.x = panels_pose[i].position.x
        p.transform.translation.y = panels_pose[i].position.y
        p.transform.translation.z = panels_pose[i].position.z

        p.transform.rotation.x = panels_pose[i].orientation.x
        p.transform.rotation.y = panels_pose[i].orientation.y
        p.transform.rotation.z = panels_pose[i].orientation.z
        p.transform.rotation.w = panels_pose[i].orientation.w

        return p

    
    def broadcast_camera_panel_transform(self, c_pose, panels_pose):
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

        p0 = self.panel_transform(panels_pose, 0)
        p1 = self.panel_transform(panels_pose, 1)
        p2 = self.panel_transform(panels_pose, 2)
        p3 = self.panel_transform(panels_pose, 3)

        self.tf_broadcaster.sendTransform([c, p0, p1, p2, p3])


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