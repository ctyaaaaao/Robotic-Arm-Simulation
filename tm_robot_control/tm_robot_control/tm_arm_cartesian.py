import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions
from sensor_msgs.msg import JointState
import sys, tty, termios

class TMArmKeyboard(Node):
    def __init__(self):
        super().__init__('tm_arm_keyboard')

        # 初始位置（笛卡爾空間）
        self.cartesian_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm_connected = False  # 預設手臂未連接

        """service client"""
        self.cli = self.create_client(SetPositions, 'set_positions')

        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Service Not Connected')
        else:
            self.get_logger().info('Service Connected')
            self.arm_connected = True

        """topic publisher"""
        self.joint_publisher = self.create_publisher(JointState, '/tm_joint_states', 10)

    def get_keyboard(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)  # 立即讀取按鍵（不需要按 Enter）
            keyboard = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return keyboard

    def run(self):
        self.get_logger().info('Moving arm to initial position...')
        self.move_arm(self.cartesian_positions)

        self.get_logger().info("""
            Enter Cartesian Position: [x, y, z, rx, ry, rz] (separated by space)
            'm': Toggle motion type (PTP_T <-> LINE_T)
            'x': Exit
        """)

        motion_types = [SetPositions.Request.PTP_T, SetPositions.Request.LINE_T]
        motion_type_index = 0  # 0: PTP_T, 1: LINE_T

        while rclpy.ok():
            self.get_logger().info("Enter new position (or press 'm' to switch mode, 'x' to exit):")
            user_input = input().strip()

            if user_input.lower() == 'x':
                self.get_logger().info('Exiting program...')
                break
            elif user_input.lower() == 'm':  # 切換運動模式
                motion_type_index = 1 - motion_type_index
                motion_name = "PTP_T" if motion_types[motion_type_index] == SetPositions.Request.PTP_T else "LINE_T"
                self.get_logger().info(f'Motion type switched to: {motion_name}')
            else:
                try:
                    new_positions = list(map(float, user_input.split()))
                    if len(new_positions) != 6:
                        raise ValueError("Invalid input format")

                    self.cartesian_positions = new_positions
                    self.get_logger().info(f'New Position: {self.cartesian_positions}')
                    self.move_arm(self.cartesian_positions, motion_type=motion_types[motion_type_index])
                except ValueError:
                    self.get_logger().warn("Invalid input! Please enter 6 floating point numbers separated by space.")

    def move_arm(self, positions, motion_type=SetPositions.Request.PTP_T, velocity=0.5, acc_time=0.2, blend_percentage=0, fine_goal=False):
        req = SetPositions.Request()
        req.motion_type = motion_type
        req.positions = positions
        req.velocity = velocity
        req.acc_time = acc_time
        req.blend_percentage = blend_percentage
        req.fine_goal = fine_goal

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if self.arm_connected:
            if future.done():
                response = future.result()
                if response and response.ok:
                    self.get_logger().info('Position updated on robotic arm successfully!')
                else:
                    self.get_logger().warn(f'Failed to update position on robotic arm. Response OK status: {response.ok if response else "No response"}')
            else:
                self.get_logger().warn('Service call to robotic arm did not complete.')
        else:
            self.get_logger().info('Operating in simulation mode, updating PyBullet only.')

        self.publish_joint_states(positions)

    def publish_joint_states(self, positions):
        msg = JointState()
        msg.position = positions
        self.joint_publisher.publish(msg)
        self.get_logger().info(f'Sent Joint State to Pybullet: {positions}')

def main(args=None):
    rclpy.init(args=args)
    node = TMArmKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
