import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class F1tenthController(Node):

    def __init__(self):
        super().__init__('f1tenth_controller')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.prepare_and_send_drive_message(1.0, 0.5, 0.5)
        # self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        range_min = msg.range_min
        range_max = msg.range_max
        ranges = msg.ranges # list of measurements
        # self.get_logger().info(f'Received measuremnt[angle_min: {angle_min}, angle_max: {angle_max}, angle_increment: {angle_increment}, range_max: {range_max}]')

    def prepare_and_send_drive_message(self, velocity, angle, acceleration):
        msg = AckermannDriveStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'f1tenth_controller'
        msg.drive.speed = velocity
        msg.drive.acceleration = acceleration
        msg.drive.jerk = 0.1
        msg.drive.steering_angle = angle

        self.get_logger().info(f"Prepared ackermann message: {msg}")
        self.publisher_.publish(msg)



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
