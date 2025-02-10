import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions
from sensor_msgs.msg import JointState
import sys, tty, termios

class TMArmKeyboard(Node):
    def __init__(self):
        super().__init__('tm_arm_keyboard')

        # 手臂各關節初始設定弧度
        # self.joint_positions = [0.35, -0.57, 0.05, -3.14, 0.0, 0.81]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm_connected = False  # 預設手臂未連接

        """service client"""
        #建立與server端相同type, name為tm_arm_set_position的client端(service server: tm_driver)
        self.cli = self.create_client(SetPositions, 'set_positions')
        # self.simulation_mode = False

        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Service Not Coonnect') 
            # self.simulation_mode = True
        else:
            self.get_logger().info('Service Connect')
            self.arm_connected = True
            self.simulation_mode = True

        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        
        """topic publisher"""
        self.joint_publisher = self.create_publisher(JointState, '/tm_joint_states', 10)

    def get_keyboard(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)#不須按enter 即可讀取輸入
            keyboard = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return keyboard
    

    def run(self):
        # joint_positions = [0.35, -0.57, 0.05, -3.14, 0.0, 0.81]
        rotate_radian = 0.035 #0.035 radian = 2 degree 
        self.get_logger().info('Moving arm to initial position...')
        self.move_arm(self.joint_positions)

        self.get_logger().info("""
            'e/d': 1st joint increase/decrease
            'r/f': 2nd joint increase/decrease
            't/g': 3rd joint increase/decrease
            'y/h': 4th joint increase/decrease
            'u/j': 5th joint increase/decrease
            'i/k': 6th joint increase/decrease 
            'x': Exit
        """)

        keyboard_to_joint_mapping = {
            'e':(0, rotate_radian), 'd':(0, -rotate_radian),
            'r':(1, rotate_radian), 'f':(1, -rotate_radian),
            't':(2, rotate_radian), 'g':(2, -rotate_radian),
            'y':(3, rotate_radian), 'h':(3, -rotate_radian),
            'u':(4, rotate_radian), 'j':(4, -rotate_radian),
            'i':(5, rotate_radian), 'k':(5, -rotate_radian),
        }

        while rclpy.ok():
            keyboard = self.get_keyboard()
            if keyboard == 'x':
                self.get_logger().info('Exiting program...')
                break
            if keyboard in keyboard_to_joint_mapping:
                joint_idx, radian_change = keyboard_to_joint_mapping[keyboard]
                self.joint_positions[joint_idx] += radian_change

                self.get_logger().info(f'{joint_idx} joint: {self.joint_positions[joint_idx]:.2f} radian')
                self.move_arm(self.joint_positions)
            else:
                continue  

    #PTP_J -> position unit: radian 
    #PTP_T, LINE_T -> position unit: cartesian [x, y, z, rx, ry, rz]
    def move_arm(self, joint_positions, motion_type=SetPositions.Request.PTP_J, velocity=0.5, acc_time=0.2, blend_perceptage=0, fine_goal=False):
        #呼叫service
        req = SetPositions.Request()
        req.motion_type = motion_type
        req.positions = joint_positions
        req.velocity = velocity
        req.acc_time = acc_time
        req.blend_percentage = blend_perceptage
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
                    # self.arm_connected = False
            else:
                self.get_logger().warn('Service call to robotic arm did not complete.')
                # self.arm_connected = False
        else:
            self.get_logger().info('Operating in simulation mode, updating PyBullet only.')

        # if future.done():
        #     response = future.result()
        #     if not response.ok:
        #         self.get_logger().warn('Connect Fail')
        #         self.arm_connected = False
        # else:
        #     self.get_logger().warn('Service Fail')
        #     self.arm_connected = False
        self.publish_joint_states(joint_positions)

    def publish_joint_states(self, joint_positions):
        msg = JointState()
        msg.position = joint_positions
        self.joint_publisher.publish(msg)
        self.get_logger().info(f'Sent Joint State to Pybullet: {joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = TMArmKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()