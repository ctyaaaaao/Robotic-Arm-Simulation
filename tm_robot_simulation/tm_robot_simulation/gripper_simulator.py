import rclpy
from rclpy.node import Node
from grpr2f85_ifaces.srv import SetGripperState
from sensor_msgs.msg import JointState
import pybullet as p
import pybullet_data

class GripperSimulator(Node):
    def __init__(self):
        super().__init__('gripper_simulator')
        
        """Subscription to joint states for simulation"""
        self.subscription = self.create_subscription(
            JointState,
            '/gripper_joint_states',
            self.update_gripper_state,
            10
        )

        """Service server to simulate gripper control"""
        self.srv = self.create_service(SetGripperState, 'set_gripper_state', self.set_gripper_callback)
        self.get_logger().info('Gripper simulator service is ready.')

        self.init_pybullet()

    def init_pybullet(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.robot_id = p.loadURDF("/home/regina/tmdriver_ws/src/robotiq_arg85_description/robots/robotiq_arg85_description.urdf", useFixedBase=True)

        # 設定夾爪關節索引，依據 URDF 檔案調整
        self.gripper_joint_indices = [1, 2]  # 夾爪左右指頭的 joint index

    def set_gripper_callback(self, request, response):
        position = request.position / 255.0  # Normalize position
        for joint_idx in self.gripper_joint_indices:
            p.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=joint_idx,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=position,
                                    force=request.force)

        response.ok = True
        response.status_code = 0
        response.result = 'Gripper moved successfully in simulation.'
        self.get_logger().info(f'Gripper moved to position: {position}')
        return response

    def update_gripper_state(self, msg):
        """Update gripper position in simulation based on joint states"""
        position = msg.position[0]
        for joint_idx in self.gripper_joint_indices:
            p.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=joint_idx,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=position)
        self.get_logger().info(f'Gripper state updated from topic: {position}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
