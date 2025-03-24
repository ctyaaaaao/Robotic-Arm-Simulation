import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tm_msgs.srv import SetPositions
import pybullet as p
import pybullet_data
import numpy as np
from geometry_msgs.msg import Pose
import time


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
        self.robot_id = p.loadURDF("/home/regina/Hardware/src/tm2_ros2/tm_description/urdf/tm12_with_gripper.urdf",
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
    
    def simulate_ai_action(self, scenario="", repeat_times=1):
        try:
            for i in range(repeat_times):
                self.get_logger().info(f"[SIM] AI Action 第 {i+1} 次")

                # 模擬拍照位置
                photo_pose = [0.4, -0.3, 0.4, -3.14, 0.0, 0.78]
                self.send_pose_to_simulator(photo_pose)
                time.sleep(1.0)

                # 模擬 feedback 當作 T_Grpr2Base
                T_Grpr2Base = self.pose_to_transform(photo_pose)

                # 模擬 AI 推論（隨機生成或用固定值）
                obj_pose_camera = [0.1, 0.0, 0.2, 0, 0, 0]  # 相對於相機
                T_Obj2Cam = self.pose_to_transform(obj_pose_camera)

                # 計算物體在 base 下的位姿
                T_Obj2Base = np.matmul(T_Grpr2Base, T_Obj2Cam)
                obj_pose_base = self.transform_to_pose(T_Obj2Base)

                self.get_logger().info(f"[SIM] 物體 base pose: {obj_pose_base}")

                # 模擬取物與放置
                self.send_pose_to_simulator(obj_pose_base)
                time.sleep(1.0)

                place_pose = [0.1, -0.5, 0.1, -3.14, 0.0, 0.78]
                self.send_pose_to_simulator(place_pose)
                time.sleep(1.0)

            # 回安全點
            home_pose = [0.0, 0.0, 0.3, 0, 0, 0]
            self.send_pose_to_simulator(home_pose)
            time.sleep(1.0)

        except Exception as e:
            self.get_logger().error(f'[SIM] 執行 AI Action 發生錯誤: {e}')

    def pose_to_transform(self, pose):
        """pose = [x, y, z, rx, ry, rz] 轉為 4x4 矩陣"""
        trans = np.eye(4)
        trans[0:3, 3] = pose[0:3]
        rot = p.getMatrixFromEuler(pose[3:6])
        trans[0:3, 0] = rot[0:3]
        trans[0:3, 1] = rot[3:6]
        trans[0:3, 2] = rot[6:9]
        return trans

    def transform_to_pose(self, matrix):
        """4x4 矩陣轉為 [x, y, z, rx, ry, rz]"""
        pos = matrix[0:3, 3]
        rot_matrix = matrix[0:3, 0:3]
        quat = p.getQuaternionFromEuler(p.getEulerFromMatrix(rot_matrix))
        euler = p.getEulerFromQuaternion(quat)
        return [*pos, *euler]
    
    def send_pose_to_simulator(self, pose):
        client = self.create_client(SetPositions, 'set_positions')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_positions 服務中...')

        request = SetPositions.Request()
        request.positions = pose
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().ok:
            self.get_logger().info(f"成功移動至 {pose}")
        else:
            self.get_logger().warn("移動失敗")


def main(args=None):
    rclpy.init(args=args)
    node = TMArmSimulator()

    # 等待 PyBullet 初始化
    time.sleep(2)
    node.simulate_ai_action(repeat_times=2)


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
