import rclpy
from rclpy.node import Node
from grpr2f85_ifaces.srv import SetGripperState
from sensor_msgs.msg import JointState
import sys, tty, termios

class GripperKeyboard(Node):
    def __init__(self):
        super().__init__('gripper_keyboard')

        """service client"""
        self.cli = self.create_client(SetGripperState, 'set_gripper_state')
        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Service Not Connected')
            self.gripper_connected = False
        else:
            self.get_logger().info('Service Connected')
            self.gripper_connected = True

        """topic publisher for simulation"""
        self.gripper_joint_publisher = self.create_publisher(JointState, '/gripper_joint_states', 10)

    def get_keyboard(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            keyboard = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return keyboard

    def gripper_run(self):
        self.get_logger().info("""
            'o': Open gripper
            'p': Close gripper
            'x': Exit
        """)

        while rclpy.ok():
            keyboard = self.get_keyboard()
            if keyboard == 'x':
                self.get_logger().info('Exiting program...')
                break
            elif keyboard == 'o':
                self.move_gripper(position=0)  # Fully open
            elif keyboard == 'p':
                self.move_gripper(position=255)  # Fully close

    def move_gripper(self, position, speed=255, force=255, wait_time=0):
        req = SetGripperState.Request()
        req.position = position
        req.speed = speed
        req.force = force
        req.wait_time = wait_time

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            response = future.result()
            if response and response.ok:
                self.get_logger().info('Gripper position updated successfully!')
            else:
                self.get_logger().warn(f'Failed to update gripper position. Status: {response.status_code}, Message: {response.result}')
        else:
            self.get_logger().warn('Service call did not complete.')

        self.publish_joint_states(position)

    def publish_joint_states(self, position):
        msg = JointState()
        msg.name = ['gripper_finger_joint']
        msg.position = [position / 255.0]  # Normalize position for simulation
        self.gripper_joint_publisher.publish(msg)
        self.get_logger().info(f'Published gripper joint state: {msg.position[0]}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperKeyboard()
    node.gripper_run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
