import pybullet as p
import pybullet_data

def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    # robot_id = p.loadURDF("/home/regina/tmdriver_ws/src/tm_description/urdf/tm12-nominal.urdf", useFixedBase=True)
    # robot_id = p.loadURDF("/home/regina/tmdriver_ws/src/robotiq_arg85_description/robots/robotiq_arg85_description.urdf", useFixedBase=True)
    robot_id = p.loadURDF("/home/regina/tmdriver_ws/src/tm_description/urdf/tm12_with_gripper.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

    return robot_id
    

if __name__ == '__main__':
    main()

    while True:
        p.stepSimulation()