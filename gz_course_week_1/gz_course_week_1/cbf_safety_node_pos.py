import rclpy
import math
import time
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Twist, PointStamped, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from .CBF_functions import rectangle_cbf, rectangle_cbf_nonholonomic, CBFsolver
class CBFSafteyNode(Node):
    def __init__(self):
        super().__init__('cbf_safety_node')
        self.cbf_x_bounds = (-2.75, 2.75)
        self.cbf_y_bounds = (-6.75, 6.75)
        self.kcbf = 5
        self.sharpness = 0.8
        self.delta_ub = 0.9
        self.ell = 0.15
        self.max_angular_speed = 2.84
        self.max_linear_speed = 1.0
        self.log_frequency = 3.0
        self.last_log_time = 0.0
        self.watchdog_timeout = 0.2
        self.last_cmd_vel_time = None
        self.create_timer(0.05, self._watchdog_check)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 1)
        self.create_subscription(Twist, '/cmd_vel', self.safe_control_input_callback, 1)
        self.safe_cmd_vel_publisher = self.create_publisher(Twist, '/safe_cmd_vel', 10)
        self.cbf_status_publisher = self.create_publisher(String, 'cbf/status', 10)
        self.current_pos_publisher = self.create_publisher(PointStamped, 'cbf/current_pos', 10)
        self.current_vel_publisher = self.create_publisher(Vector3, 'cbf/current_vel', 10)
        self.boundary_marker_pub = self.create_publisher(Marker, '/cbf/boundary', 10)
        self.wall_marker_pub = self.create_publisher(MarkerArray, '/cbf/walls', 10)
        self.current_pos = None
        self.current_orientation = None
        self.current_yaw = 0.0
        self.callback_count = 0
        self.boundary_published = False
        self.walls_published = False
        self.wall_specs = [
            {'name': 'west_wall', 'x': -2.775, 'y': 0.0, 'size_x': 0.05, 'size_y': 13.5},
            {'name': 'east_wall', 'x': 2.775, 'y': 0.0, 'size_x': 0.05, 'size_y': 13.5},
            {'name': 'north_wall', 'x': 0.0, 'y': 6.775, 'size_x': 5.5, 'size_y': 0.05},
            {'name': 'south_wall', 'x': 0.0, 'y': -6.775, 'size_x': 5.5, 'size_y': 0.05},
        ]
        self.get_logger().info("CBF Safety Filter Node started (TurtleBot).")
        self.get_logger().info(f"Regular logging rate: {self.log_frequency} Hz")
        self.create_timer(0.1, self._publish_cbf_boundary_once)
        self.create_timer(0.1, self._publish_walls_once)
    def _publish_cbf_boundary_once(self):
        if self.boundary_published:
            return
        try:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.color = ColorRGBA()
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            x_min, x_max = self.cbf_x_bounds
            y_min, y_max = self.cbf_y_bounds
            corners = [
                (x_max, y_max, 0.0),
                (x_min, y_max, 0.0),
                (x_min, y_min, 0.0),
                (x_max, y_min, 0.0),
                (x_max, y_max, 0.0),
            ]
            from geometry_msgs.msg import Point
            for x, y, z in corners:
                point = Point()
                point.x = x
                point.y = y
                point.z = z
                marker.points.append(point)
            self.boundary_marker_pub.publish(marker)
            self.boundary_published = True
            self.get_logger().info("CBF boundary marker published")
        except Exception as e:
            self.get_logger().error(f"Error publishing boundary: {str(e)}")
    def _publish_walls_once(self):
        if self.walls_published:
            return
        try:
            marker_array = MarkerArray()
            for i, wall in enumerate(self.wall_specs):
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "cbf_walls"
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = wall['x']
                marker.pose.position.y = wall['y']
                marker.pose.position.z = 0.15
                marker.pose.orientation.w = 1.0
                marker.scale.x = wall['size_x']
                marker.scale.y = wall['size_y']
                marker.scale.z = 0.3
                marker.color = ColorRGBA()
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker_array.markers.append(marker)
            self.wall_marker_pub.publish(marker_array)
            self.walls_published = True
            self.get_logger().info("Wall markers published")
        except Exception as e:
            self.get_logger().error(f"Error publishing wall markers: {str(e)}")
    def publish_cbf_status(self, status):
        status_msg = String()
        status_msg.data = status
        self.cbf_status_publisher.publish(status_msg)
    def odom_callback(self, msg: Odometry):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_orientation = np.array([
            math.cos(self.current_yaw),
            math.sin(self.current_yaw)
        ])
    def _watchdog_check(self):
        if self.last_cmd_vel_time is None:
            return
        if (time.time() - self.last_cmd_vel_time) <= self.watchdog_timeout:
            return
        self.safe_cmd_vel_publisher.publish(Twist())
        self.last_cmd_vel_time = time.time()
        self.get_logger().error(
            f"⚠️  No /cmd_vel received for >{self.watchdog_timeout}s -- nominal command "
            "source appears to have stopped publishing. Stopping the robot rather than "
            "continuing to execute a stale /safe_cmd_vel with no active safety filtering.")
    def safe_control_input_callback(self, msg: Twist):
        self.callback_count += 1
        self.last_cmd_vel_time = time.time()
        current_time = time.time()
        should_log = (current_time - self.last_log_time) >= (1.0 / self.log_frequency)
        if self.current_pos is None or self.current_orientation is None:
            if should_log:
                self.get_logger().warn("⚠️ Waiting for odometry data...")
            return
        try:
            u_nom = np.array([msg.linear.x, msg.angular.z])
            if should_log:
                self.get_logger().info(f"📍 Position: [{self.current_pos[0]:.3f}, {self.current_pos[1]:.3f}]")
                self.get_logger().info(f"📐 Yaw: {math.degrees(self.current_yaw):.1f}°")
                self.get_logger().info(f"📥 Nominal command (v, omega): [{u_nom[0]:.3f}, {u_nom[1]:.3f}]")
            f_x = np.array([0.0, 0.0])
            b_matrix_status, _ = rectangle_cbf(
                self.current_pos, self.cbf_x_bounds, self.cbf_y_bounds)
            violations = b_matrix_status > 0
            if np.any(violations):
                self.publish_cbf_status("VIOLATION")
                wall_names = ["West", "South", "East", "North"]
                violated_walls = [wall_names[i] for i, v in enumerate(violations) if v]
                if should_log:
                    self.get_logger().warn(f"❌ Barrier violations: {', '.join(violated_walls)}")
            else:
                nearest_barrier_value = np.max(b_matrix_status)
                if nearest_barrier_value > -4.0:
                    self.publish_cbf_status("WARNING")
                    if should_log:
                        self.get_logger().info("⚠️  Close to barrier boundary")
                else:
                    self.publish_cbf_status("SAFE")
                    if should_log:
                        self.get_logger().info("✅ All barriers satisfied")
            if should_log:
                self.get_logger().info(f"🛡️  Barrier values: {b_matrix_status}")
            b_matrix, grad_b = rectangle_cbf_nonholonomic(
                self.current_pos, self.current_yaw, self.ell,
                self.cbf_x_bounds, self.cbf_y_bounds)
            u_safe = CBFsolver(
                'square_rcbf', 'QP', b_matrix, grad_b, None, f_x, u_nom, None,
                self.kcbf, self.delta_ub, None)
            if u_safe is None:
                self.get_logger().error("❌ CBF solver returned None!")
                return
            if should_log:
                self.get_logger().info(
                    f"🛡️  Safe command (v, omega): [{u_safe[0]:.3f}, {u_safe[1]:.3f}]")
                self.last_log_time = current_time
            safe_twist = Twist()
            safe_twist.linear.x = float(
                np.clip(u_safe[0], -self.max_linear_speed, self.max_linear_speed))
            safe_twist.angular.z = float(
                np.clip(u_safe[1], -self.max_angular_speed, self.max_angular_speed))
            self.safe_cmd_vel_publisher.publish(safe_twist)
        except Exception as e:
            self.get_logger().error(f"💥 Exception in CBF callback: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
def main(args=None):
    rclpy.init(args=args)
    cbf_safety_node = CBFSafteyNode()
    rclpy.spin(cbf_safety_node)
    cbf_safety_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()