import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tm_msgs.srv import SetPositions
import pybullet as p
import pybullet_data

class TMArmSimulator(Node):
    def __init__(self):
        super().__init__('tm_arm_simulator')
        """topic subscriber"""
        self.subscription = self.create_subscription(
            JointState,
            '/tm_joint_states',
            self.joint_callback,
            10
        )

        """Simulating service server to receive service clients"""
        self.srv = self.create_service(SetPositions, 'set_positions', self.set_positions_callback)

        self.init_pybullet()

    def init_pybullet(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        # self.robot_id = p.loadURDF("/home/regina/tmdriver_ws/src/tm_description/urdf/tm12-nominal.urdf", useFixedBase=True)
        self.robot_id = p.loadURDF("/home/regina/tmdriver_ws/src/tm_description/urdf/tm12_with_gripper.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        
        # 設定初始關節位置
        initial_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        for i, joint_position in enumerate(initial_joint_positions):
            p.resetJointState(self.robot_id, i, joint_position)

        p.stepSimulation()
        self.get_logger().info('Initialized arm to default joint positions.')

    def joint_callback(self, msg):
        """Subscribing /tm_joint_states"""
        for i, joint_position in enumerate(msg.position):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition = joint_position
            )
        p.stepSimulation()
        self.get_logger().info(f'Simulating Joint States: {msg.position}')

    def set_positions_callback(self, request, response):
        # Simulating service calls
        self.get_logger().info(f"Received service request to move to positions: {request.positions}")
        
        for i, joint_position in enumerate(request.positions):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=joint_position
            )
        p.stepSimulation()

        response.ok = True  
        return response

def main(args=None):
        rclpy.init(args=args)
        node = TMArmSimulator()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()