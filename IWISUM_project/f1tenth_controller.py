import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

def fuzzy_logic_model(lidar_msg):
    angle_min = lidar_msg.angle_min
    angle_max = lidar_msg.angle_max
    angle_increment = lidar_msg.angle_increment
    range_min = lidar_msg.range_min
    range_max = lidar_msg.range_max
    ranges = lidar_msg.ranges # list of measurements

    result = {'velocity': 1, 'acceleration': 0.1, 'steering_wheel_angle': 0.5}
    return result

class F1tenthController(Node):

    def __init__(self):
        super().__init__('f1tenth_controller')
        self.last_lidar_record = None
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)

        timer_period_s = 0.1  # seconds
        self.timer = self.create_timer(timer_period_s, self.timer_callback)

    def lidar_callback(self, msg):
        self.last_lidar_record = msg

    def prepare_and_send_drive_message(self, velocity, angle, acceleration):
        msg = AckermannDriveStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'f1tenth_controller'
        msg.drive.speed = float(velocity)
        msg.drive.acceleration = float(acceleration)
        msg.drive.jerk = 0.1
        msg.drive.steering_angle = float(angle)

        self.get_logger().info(f"Prepared ackermann message: {msg}")
        self.publisher_.publish(msg)

    def timer_callback(self):
        if self.last_lidar_record != None:
            self.get_logger().info('Processing lidar records')
            result = fuzzy_logic_model(self.last_lidar_record)
            print(result)
            self.prepare_and_send_drive_message(result['velocity'], result['steering_wheel_angle'], result['acceleration'])



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
