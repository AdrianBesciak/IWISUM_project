import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

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
        self.driving_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.reset_position_publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.i = 0

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
        self.driving_publisher.publish(msg)

    def timer_callback(self):
        if self.last_lidar_record != None:
            self.get_logger().info('Processing lidar records')
            result = fuzzy_logic_model(self.last_lidar_record)
            print(result)
            self.prepare_and_send_drive_message(result['velocity'], result['steering_wheel_angle'], result['acceleration'])
            if self.i % 50 == 0:
                self.get_logger().info(f"Sending reset position message")
                self.reset_position()
            self.i += 1

    def reset_position(self):
        initialpose_msg = PoseWithCovarianceStamped()
        initialpose_msg.pose.pose.position.x = 0.0
        initialpose_msg.pose.pose.position.y = 0.0
        initialpose_msg.pose.pose.position.z = 0.0
        initialpose_msg.pose.pose.orientation.x = 0.4
        initialpose_msg.pose.pose.orientation.y = 0.4
        initialpose_msg.pose.pose.orientation.z = 0.0
        initialpose_msg.pose.pose.orientation.w = 1.0
        self.reset_position_publisher.publish(initialpose_msg)



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
