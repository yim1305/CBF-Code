import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gz.transport13 import Node as GzTransportNode
from gz.msgs10.pose_v_pb2 import Pose_V
class GroundTruthPoseNode(Node):
    def __init__(self):
        super().__init__('ground_truth_pose_node')
        self.declare_parameter('world_name', 'NCR_Indoor_Lab')
        self.declare_parameter('entity_name', 'tb3')
        self.world_name = self.get_parameter('world_name').value
        self.entity_name = self.get_parameter('entity_name').value
        self.odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.gz_node = GzTransportNode()
        topic = f'/world/{self.world_name}/pose/info'
        if not self.gz_node.subscribe(Pose_V, topic, self._gz_pose_callback):
            self.get_logger().error(f"Failed to subscribe to gz-transport topic {topic}")
        self.get_logger().info(
            f"Ground Truth Pose Node started, bridging entity '{self.entity_name}' "
            f"from {topic} to /ground_truth/odom")
    def _gz_pose_callback(self, msg):
        for pose in msg.pose:
            if pose.name != self.entity_name:
                continue
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x = pose.position.x
            odom.pose.pose.position.y = pose.position.y
            odom.pose.pose.position.z = pose.position.z
            odom.pose.pose.orientation.x = pose.orientation.x
            odom.pose.pose.orientation.y = pose.orientation.y
            odom.pose.pose.orientation.z = pose.orientation.z
            odom.pose.pose.orientation.w = pose.orientation.w
            self.odom_pub.publish(odom)
            return
def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()