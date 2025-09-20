import rclpy
from rclpy.node import Node
from tr_messages.msg import SimGroundTruth

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")
        self.sub = self.create_subscription(SimGroundTruth, 'simulation/ground_truth', self.msgCallback, 10) # callback function -> after execution

    def msgCallback(self, msg): # receive input then get msg
        self.get_logger().info(f"Received primary robot chassis pose: x={msg.primary_robot.chassis_pose.position.x}, y={msg.primary_robot.chassis_pose.position.y}, z={msg.primary_robot.chassis_pose.position.z}")

def main():
    rclpy.init()
    node = SimpleSubscriber()
    rclpy.spin(node) # keep up and running
    node.destroy_node() # when terminated aka ctrl c
    rclpy.shutdown() 

if __name__ == '__main__':
    main()