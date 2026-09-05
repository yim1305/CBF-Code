import sys
import termios
import threading
import tty
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84
LIN_VEL_STEP = 0.01
ANG_VEL_STEP = 0.1
PUBLISH_RATE_HZ = 20.0
def clamp(value, limit):
    return max(-limit, min(limit, value))
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.quit_requested = False
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_velocity)
    def _publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = self.angular_vel
        self.publisher_.publish(twist)
def read_keys(node, settings):
    while rclpy.ok() and not node.quit_requested:
        key = get_key(settings)
        if key == 'w':
            node.linear_vel = clamp(node.linear_vel + LIN_VEL_STEP, MAX_LIN_VEL)
        elif key == 'x':
            node.linear_vel = clamp(node.linear_vel - LIN_VEL_STEP, MAX_LIN_VEL)
        elif key == 'a':
            node.angular_vel = clamp(node.angular_vel + ANG_VEL_STEP, MAX_ANG_VEL)
        elif key == 'd':
            node.angular_vel = clamp(node.angular_vel - ANG_VEL_STEP, MAX_ANG_VEL)
        elif key == 's':
            node.linear_vel = 0.0
            node.angular_vel = 0.0
        elif key == 'q' or key == '\x03':
            node.linear_vel = 0.0
            node.angular_vel = 0.0
            node.quit_requested = True
            break
        print(
            f'\rv = {node.linear_vel:+.2f} m/s   w = {node.angular_vel:+.2f} rad/s   ',
            end=''
        )
def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopKeyboard()
    print(INSTRUCTIONS)
    key_thread = threading.Thread(target=read_keys, args=(node, settings), daemon=True)
    key_thread.start()
    try:
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.publisher_.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()