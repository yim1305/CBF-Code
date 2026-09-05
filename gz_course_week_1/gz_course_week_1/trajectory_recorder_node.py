import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
class TrajectoryRecorderNode(Node):
    RECORD_HZ = 5.0
    def __init__(self):
        super().__init__('trajectory_recorder_node')
        self.actual_path = Path()
        self.actual_path.header.frame_id = "odom"
        self._last_record_time = None
        self.actual_path_pub = self.create_publisher(Path, '/waypoint/actual_path', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 1)
        self.create_timer(1.0 / self.RECORD_HZ, self._publish_path)
        self.get_logger().info("Trajectory Recorder Node started")
    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()
        if (self._last_record_time is not None
                and (now - self._last_record_time).nanoseconds < (1e9 / self.RECORD_HZ)):
            return
        self._last_record_time = now
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = msg.header.stamp
        pose.pose = msg.pose.pose
        self.actual_path.poses.append(pose)
        self.actual_path.header.stamp = msg.header.stamp
    def _publish_path(self):
        if self.actual_path.poses:
            self.actual_path_pub.publish(self.actual_path)
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()