import math
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from gz.transport13 import Node as GzTransportNode
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
class SpiralTrackerNode(Node):
    def __init__(self):
        super().__init__('spiral_tracker_node')
        self.declare_parameter('radial_rate', 0.06)
        self.declare_parameter('angular_rate', 0.24)
        self.declare_parameter('k1', 1.0)
        self.declare_parameter('lookahead', 0.15)
        self.declare_parameter('tracking_error_max', 1.0)
        self.declare_parameter('stall_timeout', 3.0)
        self.declare_parameter('stall_escape_scale', 1.0)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('max_linear', 50.0)
        self.declare_parameter('max_angular', 50.0)
        self.declare_parameter('wall_x_bounds', [-2.75, 2.75])
        self.declare_parameter('wall_y_bounds', [-6.75, 6.75])
        self.declare_parameter('wall_touch_margin', 0.05)
        self.declare_parameter('world_name', 'NCR_Indoor_Lab')
        self.radial_rate = self.get_parameter('radial_rate').value
        self.angular_rate = self.get_parameter('angular_rate').value
        self.k1 = self.get_parameter('k1').value
        self.lookahead = self.get_parameter('lookahead').value
        self.tracking_error_max = self.get_parameter('tracking_error_max').value
        self.stall_timeout = self.get_parameter('stall_timeout').value
        self.stall_escape_scale = self.get_parameter('stall_escape_scale').value
        self.center = np.array([
            self.get_parameter('center_x').value,
            self.get_parameter('center_y').value,
        ])
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.wall_x_bounds = tuple(self.get_parameter('wall_x_bounds').value)
        self.wall_y_bounds = tuple(self.get_parameter('wall_y_bounds').value)
        self.wall_touch_margin = self.get_parameter('wall_touch_margin').value
        self.world_name = self.get_parameter('world_name').value
        self.current_pos = None
        self.current_yaw = 0.0
        self.virtual_t = 0.0
        self.last_tick_time = None
        self.stall_start_time = None
        self.gz_node = GzTransportNode()
        self.touched_wall = False
        self.touch_angle = None
        x_min, x_max = self.wall_x_bounds
        y_min, y_max = self.wall_y_bounds
        farthest_wall_dist = max(
            x_max - self.center[0], self.center[0] - x_min,
            y_max - self.center[1], self.center[1] - y_min)
        marker_time_horizon = (
            farthest_wall_dist / self.radial_rate + 2 * math.pi / self.angular_rate)
        self.reference_path_marker = self._build_reference_path_marker(marker_time_horizon)
        self.reference_path_pub = self.create_publisher(
            Marker, '/spiral_tracker/reference_path', 10)
        self.create_timer(5.0, self._publish_reference_path)
        self.reference_point_pub = self.create_publisher(
            Marker, '/spiral_tracker/reference_point', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.02, self.control_tick)
        self.get_logger().info(
            f"Spiral tracker started: radial_rate={self.radial_rate}, "
            f"angular_rate={self.angular_rate}, k1={self.k1}, "
            f"lookahead={self.lookahead}, center={tuple(self.center)}, "
            f"wall_touch_margin={self.wall_touch_margin}, "
            f"tracking_error_max={self.tracking_error_max}, "
            f"stall_timeout={self.stall_timeout}, stall_escape_scale={self.stall_escape_scale}. "
            "Publishing nominal commands to /cmd_vel -- do not also run teleop_keyboard.")
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_pos = np.array([p.x, p.y])
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    def spiral_reference(self, t):
        radius = self.radial_rate * t
        angle = self.angular_rate * t
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        xd = self.center + radius * np.array([cos_a, sin_a])
        xd_dot = np.array([
            self.radial_rate * cos_a - radius * self.angular_rate * sin_a,
            self.radial_rate * sin_a + radius * self.angular_rate * cos_a,
        ])
        return xd, xd_dot
    def _build_reference_path_marker(self, time_horizon):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.ns = "spiral_reference"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)
        num_points = 300
        for t in np.linspace(0.0, time_horizon, num_points):
            xd, _ = self.spiral_reference(t)
            marker.points.append(Point(x=float(xd[0]), y=float(xd[1]), z=0.0))
        return marker
    def _publish_reference_path(self):
        self.reference_path_marker.header.stamp = self.get_clock().now().to_msg()
        self.reference_path_pub.publish(self.reference_path_marker)
    def _publish_reference_point(self, xd):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "spiral_reference_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(xd[0])
        marker.pose.position.y = float(xd[1])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)
        self.reference_point_pub.publish(marker)
    def _touching_wall(self, pos):
        x_min, x_max = self.wall_x_bounds
        y_min, y_max = self.wall_y_bounds
        m = self.wall_touch_margin
        return (pos[0] <= x_min + m or pos[0] >= x_max - m
                or pos[1] <= y_min + m or pos[1] >= y_max - m)
    def _reset_to_center(self):
        req = WorldControl()
        req.reset.all = True
        result, response = self.gz_node.request(
            f'/world/{self.world_name}/control', req, WorldControl, Boolean, 1000)
        if not result or not response.data:
            self.get_logger().error(
                f"World reset to center FAILED (result={result}) -- robot will keep "
                "spiraling from its current position instead of restarting from center.")
            return
        self.cmd_pub.publish(Twist())
        self.touched_wall = False
        self.touch_angle = None
        self.virtual_t = 0.0
        self.last_tick_time = None
        self.stall_start_time = None
        self.get_logger().info("Completed 1 lap after touching the wall -- reset robot to center.")
    def control_tick(self):
        if self.current_pos is None:
            return
        now = self.get_clock().now()
        if self.last_tick_time is None:
            dt_real = 0.0
        else:
            dt_real = (now - self.last_tick_time).nanoseconds * 1e-9
        self.last_tick_time = now
        angle = self.angular_rate * self.virtual_t
        xd, xd_dot = self.spiral_reference(self.virtual_t)
        self._publish_reference_point(xd)
        yaw = self.current_yaw
        ell = self.lookahead
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        p_l = self.current_pos + ell * np.array([cos_y, sin_y])
        error = float(np.linalg.norm(xd - p_l))
        raw_time_scale = max(0.0, min(1.0, 1.0 - error / self.tracking_error_max))
        if raw_time_scale > 0.0:
            self.stall_start_time = None
            time_scale = raw_time_scale
        else:
            if self.stall_start_time is None:
                self.stall_start_time = now
            stalled_for = (now - self.stall_start_time).nanoseconds * 1e-9
            if stalled_for >= self.stall_timeout:
                time_scale = self.stall_escape_scale
            else:
                time_scale = 0.0
        self.virtual_t += dt_real * time_scale
        if not self.touched_wall:
            if self._touching_wall(xd):
                self.touched_wall = True
                self.touch_angle = angle
                self.get_logger().info(
                    "Reference reached a wall -- will reset to center after 1 more lap.")
        elif angle - self.touch_angle >= 2 * math.pi:
            self._reset_to_center()
            return
        p_l_dot_desired = xd_dot * time_scale + self.k1 * (xd - p_l)
        v = cos_y * p_l_dot_desired[0] + sin_y * p_l_dot_desired[1]
        omega = (-sin_y * p_l_dot_desired[0] + cos_y * p_l_dot_desired[1]) / ell
        cmd = Twist()
        cmd.linear.x = float(np.clip(v, -self.max_linear, self.max_linear))
        cmd.angular.z = float(np.clip(omega, -self.max_angular, self.max_angular))
        self.cmd_pub.publish(cmd)
def main(args=None):
    rclpy.init(args=args)
    node = SpiralTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()