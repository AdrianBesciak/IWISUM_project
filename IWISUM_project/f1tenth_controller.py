import time

import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs import msg as gmsg
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from .follow import follow

# import stable_baselines3 as sb3
# model = sb3.PPO.load("f110-model")


class F1tenthController(Node):
    def __init__(self, timer_period_s: float = 0.1):
        super().__init__("f1tenth_controller")
        self.get_logger().info("Controller started!")

        self._last_lidar_record: LaserScan | None = None
        self._drive_publisher = self.create_publisher(
            AckermannDriveStamped, "drive", 10
        )
        self._reset_position_publisher = self.create_publisher(
            gmsg.PoseWithCovarianceStamped, "initialpose", 10
        )

        self._subscription = self.create_subscription(
            LaserScan, "scan", self._lidar_callback, 10
        )
        self._timer = self.create_timer(timer_period_s, self.timer_callback)
        self.reset_position()

    def _lidar_callback(self, msg: LaserScan):
        self._last_lidar_record = msg

        if self.collides(msg):
            self._handle_collision()

    @staticmethod
    def collides(msg: LaserScan, collision_threshold: float = 0.2):
        return any(dist < collision_threshold for dist in msg.ranges)

    def _handle_collision(self):
        self.get_logger().info("Collision detected! Sending reset position message")
        self.reset_position()
        time.sleep(1)

    def prepare_and_send_drive_message(self, drive: AckermannDrive):
        msg = AckermannDriveStamped(
            header=Header(
                stamp=self.get_clock().now().to_msg(), frame_id="f1tenth_controller"
            ),
            drive=drive,
        )
        self._drive_publisher.publish(msg)

    def timer_callback(self):
        if self._last_lidar_record is not None:
            scan = self._last_lidar_record

            # Follow the Gap
            speed, steering_angle = follow(scan)

            # RL Model
            # action, _states = model.predict(np.array(scan.ranges))
            # speed, steering_angle = action

            drive = AckermannDrive(
                speed=speed,
                steering_angle=steering_angle,
            )

            self.prepare_and_send_drive_message(drive=drive)

    def reset_position(self):
        initialpose_msg = gmsg.PoseWithCovarianceStamped(
            pose=gmsg.PoseWithCovariance(
                pose=gmsg.Pose(
                    position=gmsg.Point(x=0.0, y=0.0, z=0.0),
                    orientation=gmsg.Quaternion(x=0.4, y=0.4, z=0.0, w=1.0),
                )
            )
        )
        self._reset_position_publisher.publish(initialpose_msg)

        drive = AckermannDrive(
            speed=0.0,
            steering_angle=0.0,
        )
        self.prepare_and_send_drive_message(drive=drive)


def main(args=None):
    rclpy.init(args=args)

    f1tenth_controller = F1tenthController()

    rclpy.spin(f1tenth_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    f1tenth_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
