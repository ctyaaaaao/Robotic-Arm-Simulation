import pybullet as p
import pybullet_data
import os
import math

def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    script_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(script_dir, "../../tm_description/urdf/tm12_with_gripper.urdf")
    urdf_path = os.path.normpath(urdf_path)

    robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)


    #Check joint 是否為固定
    # num_joints = p.getNumJoints(robot_id)
    # print("Number of joints:", num_joints)

    # for i in range(p.getNumJoints(robot_id)):
    #     info = p.getJointInfo(robot_id, i)
    #     joint_name = info[1].decode()
    #     joint_type = info[2]
    #     print(f"Joint {i}: name={joint_name}, type={joint_type}")

    # 設定手臂角度
    arm_joint_positions = [-math.pi/2, -0.5, 1.0, 1.0, 1.5, 0.81]
    # 0 is based (fixed)
    arm_joint_indices = [1, 2, 3, 4, 5, 6]

    for i, joint_index in enumerate(arm_joint_indices):
        p.resetJointState(robot_id, joint_index, arm_joint_positions[i])


    return robot_id
    

if __name__ == '__main__':
    main()

    while True:
        p.stepSimulation()