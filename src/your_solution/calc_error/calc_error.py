import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np

class RobotListener(Node):
    def __init__(self):
        super().__init__('robot_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_x = self.create_publisher(Float64, 'your_solution/x_err', 1)
        self.publisher_y = self.create_publisher(Float64, 'your_solution/y_err', 1)
        self.publisher_z = self.create_publisher(Float64, 'your_solution/z_err', 1)

        self.frequency = 1.0
        self.timer = self.create_timer(self.frequency, self.timerCallback)

    def timerCallback(self):
        try:
            detected_transform = self.tf_buffer.lookup_transform(
                'map',
                'detected_panel',
                rclpy.time.Time()
            )

            panel_0_transform = self.tf_buffer.lookup_transform(
                'map',
                'panel_0',
                rclpy.time.Time()
            )

            panel_1_transform = self.tf_buffer.lookup_transform(
                'map',
                'panel_1',
                rclpy.time.Time()
            )

            panel_2_transform = self.tf_buffer.lookup_transform(
                'map',
                'panel_2',
                rclpy.time.Time()
            )
            panel_3_transform = self.tf_buffer.lookup_transform(
                'map',
                'panel_3',
                rclpy.time.Time()
            )

            def error(panel):
                return [detected_transform.transform.translation.x - panel.transform.translation.x, detected_transform.transform.translation.y - panel.transform.translation.y, detected_transform.transform.translation.z - panel.transform.translation.z]

            errors = [error(panel_0_transform), error(panel_1_transform), error(panel_2_transform), error(panel_3_transform)]
            norms = [np.linalg.norm(e) for e in errors] 
            min_index = np.argmin(norms)
            error_x, error_y, error_z = [component for component in errors[min_index]]

            msg_x = Float64()
            msg_x.data = error_x

            msg_y = Float64()
            msg_y.data = error_y

            msg_z = Float64()
            msg_z.data = error_z

            self.publisher_x.publish(msg_x)
            self.publisher_y.publish(msg_y)
            self.publisher_z.publish(msg_z)

        except Exception as e:
            self.get_logger().warn(str(e))

def main(args=None):
    rclpy.init(args=args)

    robot_listener = RobotListener()

    try:
        rclpy.spin(robot_listener)
    except KeyboardInterrupt:
        pass

    robot_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()