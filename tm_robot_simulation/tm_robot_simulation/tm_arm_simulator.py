import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tm_msgs.srv import SetPositions
import pybullet as p
import pybullet_data

class TMArmSimulator(Node):
    def __init__(self):
        super().__init__('tm_arm_simulator')

        """訂閱 /tm_joint_states (如果還需要支援關節座標控制)"""
        self.subscription = self.create_subscription(
            JointState,
            '/tm_joint_states',
            self.joint_callback,
            10
        )

        """創建 service server 來接收 Cartesian Position (x, y, z, rx, ry, rz)"""
        self.srv = self.create_service(SetPositions, 'set_positions', self.set_positions_callback)

        self.init_pybullet()

    def init_pybullet(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        # 載入機械臂模型
        self.robot_id = p.loadURDF("/home/regina/tmdriver_ws/src/tm_description/urdf/tm12_with_gripper.urdf",
                                   useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

        # 設定初始關節位置
        initial_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        for i, joint_position in enumerate(initial_joint_positions):
            p.resetJointState(self.robot_id, i, joint_position)

        p.stepSimulation()
        self.get_logger().info('Initialized arm to default joint positions.')

    def joint_callback(self, msg):
        """從 /tm_joint_states 訂閱 JointState (如果還需要關節座標控制)"""
        for i, joint_position in enumerate(msg.position):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=joint_position
            )
        p.stepSimulation()
        self.get_logger().info(f'Simulating Joint States: {msg.position}')

    def set_positions_callback(self, request, response):
        """處理 Cartesian Position (x, y, z, rx, ry, rz)，透過逆運動學轉換成關節角度"""
        self.get_logger().info(f"Received Cartesian Position: {request.positions}")

        # 解析輸入的座標
        x, y, z, rx, ry, rz = request.positions

        # 計算逆運動學 (IK) 來獲取關節角度
        joint_positions = p.calculateInverseKinematics(self.robot_id, 
                                                       endEffectorLinkIndex=5,  # 假設第5個 joint 是手臂末端
                                                       targetPosition=[x, y, z],
                                                       targetOrientation=p.getQuaternionFromEuler([rx, ry, rz]))

        # 檢查是否計算成功
        if joint_positions:
            # 發送新的關節位置
            for i, joint_position in enumerate(joint_positions[:6]):  # 只取前6個關節
                p.setJointMotorControl2(
                    self.robot_id,
                    i,
                    p.POSITION_CONTROL,
                    targetPosition=joint_position
                )
            
            p.stepSimulation()
            self.get_logger().info(f'Updated joint positions: {joint_positions[:6]}')
            response.ok = True
        else:
            self.get_logger().warn("Failed to compute inverse kinematics!")
            response.ok = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TMArmSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
