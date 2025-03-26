import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tm_msgs.srv import SetPositions
import pybullet as p
import pybullet_data
import numpy as np
from geometry_msgs.msg import Pose
import time
import cv2
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import math

class TMArmSimulator(Node):
    def __init__(self):
        super().__init__('tm_arm_simulator')
        self.load_camera_config()

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

    def load_camera_config(self):
        config_file = os.path.join(
            get_package_share_directory('tm12_amm'),
            'config',
            'camera_config.yaml'
        )

        with open(config_file, 'r') as f:
            data = yaml.safe_load(f)

        cam_params = data['/**']['ros__parameters']['camera']
        hand_eye_params = data['/**']['ros__parameters']['hand_eye']

        self.camera_matrix_ = np.array(cam_params['matrix']).reshape((3, 3))
        self.dist_coeffs_ = np.array(cam_params['distortion'])
        self.hand_eye_rotation_ = np.array(hand_eye_params['rotation']).reshape((3, 3))
        self.hand_eye_translation_ = np.array(hand_eye_params['translation']).reshape((3, 1))

        self.get_logger().info(f"Loaded camera_matrix:\n{self.camera_matrix_}")
        self.get_logger().info(f"Loaded distortion:\n{self.dist_coeffs_}")
        self.get_logger().info(f"Loaded hand-eye:\n{self.hand_eye_rotation_}, {self.hand_eye_translation_}")

    def init_pybullet(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        self.ee_link_index = 5  # TM12 通常是末端執行器，或你可以用 p.getNumJoints(robot_id) 測


        cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=[1, 0, 0, 1])
        cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02])
        self.cube_id = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=cube_collision,
            baseVisualShapeIndex=cube_visual,
            basePosition=[0.3, -0.3, 0.15]
        )

        # 載入機械臂模型
        self.robot_id = p.loadURDF("/home/regina/Hardware/src/tm2_ros2/tm_description/urdf/tm12_with_gripper.urdf",
                                   useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

        # 設定初始關節位置
        initial_joint_positions = [-math.pi/2, -0.5, 1.0, 1.0, 1.5, 0.81] 
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
                # photo_pose = [0.4, -0.3, 0.4, -3.14, 0.0, 0.78]
                # self.send_pose_to_simulator(photo_pose)
                # time.sleep(1.0)

                # 在拍照 pose 定位完成後加上這段，模擬裝在手臂末端的相機
                link_state = p.getLinkState(self.robot_id, self.ee_link_index)
                eye_position = link_state[0]  # 世界座標中的相機位置
                orientation = link_state[1]   # 四元數

                # 將四元數轉為朝向向量（相機正前方）
                rot_matrix = p.getMatrixFromQuaternion(orientation)
                forward_vec = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
                target_position = [eye_position[i] + forward_vec[i]*0.1 for i in range(3)]


                # 拍照
                # eye = [0.4, -0.3, 0.4]
                # target = [0.5, -0.3, 0.3]
                # rgb_img, depth_img = self.get_virtual_camera_images(eye, target)
                rgb_img, depth_img = self.get_virtual_camera_images(eye_position, target_position)
                self.get_logger().info(f"[SIM] 拍照完成 - RGB: {rgb_img.shape}, Depth: {depth_img.shape}")


                # 模擬 feedback 當作 T_Grpr2Base
                # T_Grpr2Base = self.pose_to_transform(photo_pose)
                T_Grpr2Base = self.get_transform_from_link(self.ee_link_index)


                # 模擬 AI 推論（隨機生成或用固定值）
                # obj_pose_camera = [0.1, 0.0, 0.2, 0, 0, 0]  # 相對於相機
                obj_pose_camera = self.fake_ai_model()

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

    def fake_ai_model(self):
        """模擬 AI 偵測 cube 並返回其位置（相機座標系下）"""
        # 因為你知道 cube 被放在 [0.5, -0.3, 0.2]（世界座標）
        # 而相機在 [0.4, -0.3, 0.4] 看向 +x
        # 你可以手動推個座標，也可以反算
        return [0.1, 0.0, 0.2, 0, 0, 0]  # 距相機 0.1m 前方


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

    def get_virtual_camera_images(self, eye_position, target_position):
        """從 PyBullet 虛擬相機拍攝 RGB 和 Depth 影像"""
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=eye_position,
            cameraTargetPosition=target_position,
            cameraUpVector=[0, 0, 1]
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60.0, aspect=640/480, nearVal=0.01, farVal=1.5
        )
        width, height, rgba_img, depth_img, _ = p.getCameraImage(
            width=640,
            height=480,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # 處理影像資料
        rgb_np = np.reshape(rgba_img, (480, 640, 4))[:, :, :3]  # 取 RGB，不要 alpha
        depth_np = np.reshape(depth_img, (480, 640))

        cv2.imwrite("rgb_sim.png", cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR))
        cv2.imwrite("depth_sim.png", (depth_np * 255).astype(np.uint8))

        return rgb_np, depth_np

    def get_transform_from_link(self, link_index):
        pos, orn = p.getLinkState(self.robot_id, link_index)[:2]
        T = np.eye(4)
        T[0:3, 3] = pos
        rot = p.getMatrixFromQuaternion(orn)
        T[0:3, 0] = rot[0:3]
        T[0:3, 1] = rot[3:6]
        T[0:3, 2] = rot[6:9]
        return T


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
