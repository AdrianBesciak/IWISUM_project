import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class F1tenthController(Node):

    def __init__(self):
        super().__init__('f1tenth_controller')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        # self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        range_min = msg.range_min
        range_max = msg.range_max
        ranges = msg.ranges # list of measurements
        self.get_logger().info(f'Received measuremnt[angle_min: {angle_min}, angle_max: {angle_max}, angle_increment: {angle_increment}, range_max: {range_max}]')


def main(args=None):
    rclpy.init(args=args)

    f1tenth_controller = F1tenthController()

    rclpy.spin(f1tenth_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    f1tenth_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
