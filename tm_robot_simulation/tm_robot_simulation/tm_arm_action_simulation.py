import pybullet as p
import pybullet_data
import os
import time
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory


# -------------------- 基本控制函式 --------------------
def move_to_pose(robot_id, joint_indices, joint_positions):
    for i, joint_index in enumerate(joint_indices):
        p.resetJointState(robot_id, joint_index, joint_positions[i])

def pick_at(robot_id, joint_indices, pose):
    print(f"[PICK] 模擬移動到 {pose}")
    move_to_pose(robot_id, joint_indices, pose)

def place_at(robot_id, joint_indices, pose):
    print(f"[PLACE] 模擬移動到 {pose}")
    move_to_pose(robot_id, joint_indices, pose)

# -------------------- 模擬 AI 偵測回傳 --------------------
def mock_ai_detect_object():
    trans = np.eye(4)
    trans[0, 3] = 0.1
    trans[2, 3] = 0.1
    return trans

def get_transform_from_pose(pose):
    trans = np.eye(4)
    trans[0, 3] = pose[0]
    trans[1, 3] = pose[1]
    trans[2, 3] = pose[2]
    return trans

def pose_from_transform(transform):
    return [transform[0,3], transform[1,3], transform[2,3], -3.14, 0., 0.78]

# -------------------- 主流程 --------------------
def sim_ai_action(robot_id, arm_joint_indices, pose_take_photo, place_pose, repeat_times=1):
    for i in range(repeat_times):
        print(f"\n--- 模擬第 {i+1} 次取放 ---")

        move_to_pose(robot_id, arm_joint_indices, pose_take_photo)
        print("[✓] 已移動到拍照位置")

        T_Grpr2Base = get_transform_from_pose(pose_take_photo)
        T_Obj2Cam = mock_ai_detect_object()
        T_Obj2Base = T_Grpr2Base @ T_Obj2Cam
        pose_obj2base = pose_from_transform(T_Obj2Base)

        print(f"[✓] 模擬 AI 偵測位姿 (base): {pose_obj2base}")

        pick_at(robot_id, arm_joint_indices, pose_obj2base)
        place_at(robot_id, arm_joint_indices, place_pose)

    print("\n[✓] 所有操作完成，回到原點")
    move_to_pose(robot_id, arm_joint_indices, [0, 0, 0, 0, 0, 0])

# -------------------- 初始化與載入 --------------------
def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # urdf_path = os.path.join(
    #     get_package_share_directory('tm_description'),
    #     'urdf', 'tm12_with_gripper.urdf'
    # )
    # print(f"[DEBUG] 嘗試載入 URDF: {urdf_path}")
    urdf_path = os.path.expanduser("/home/regina/Hardware/src/tm2_ros2/tm_description/urdf/tm12_with_gripper.urdf")


    robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

    arm_joint_positions = [-math.pi/2, -0.5, 1.0, 1.0, 1.5, 0.81]
    arm_joint_indices = [1, 2, 3, 4, 5, 6]
    move_to_pose(robot_id, arm_joint_indices, arm_joint_positions)

    pose_take_photo = arm_joint_positions
    place_pose = [0.1, -0.5, 0.1, -3.14, 0., 0.7854]

    sim_ai_action(robot_id, arm_joint_indices, pose_take_photo, place_pose, repeat_times=1)

    while True:
        p.stepSimulation()
        time.sleep(1. / 240)

if __name__ == '__main__':
    main()
