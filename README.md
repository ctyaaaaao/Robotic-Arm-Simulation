# TM Robot Simulation Setup and Control
Controlling and simulating  TM12 with 2F-85 gripper based on pybullet and ros2

## Directory Structure
Make sure your URDF path and files are correctly set in the following location:
```
/tm_robot_simulation/tm_robot_simulation/tm_arm_simulator.py
```

## Building the Packages
To build the necessary packages, use the following command:
```bash
colcon build --packages-select tm_robot_control tm_robot_simulation
```

## Launching the Simulation and Driver
First, bring up the TM driver:
```bash
ros2 launch tm_driver tm_bringup.launch.py
```

Then, launch the TM robot simulation:
```bash
ros2 launch tm_robot_simulation tm_robot_simulation.launch.py
```

## Running the Keyboard Control
To control the robotic arm using the keyboard, run:
```bash
ros2 run tm_robot_control tm_arm_keyboard
```

## Verifying Nodes and Topics
### List Active Nodes
Ensure the following nodes are active:
```bash
ros2 node list
```
Expected output should include:
- `/tm_driver_node`
- `/tm_arm_keyboard`
- `/tm_arm_simulator`

### List Active Topics
Check that the following topic is published:
```bash
ros2 topic list
```
Expected topic:
- `/tm_joint_states`

### List Available Services
Ensure the following service is available:
```bash
ros2 service list
```
Expected service:
- `/set_positions`

## Debugging and Monitoring Tools
### Visualize Node Connections
To visualize the connections between nodes, use:
```bash
ros2 run rqt_graph rqt_graph
```

### Monitor Topic Information
To display detailed information about current topics, including name, type, bandwidth, frequency, and values:
```bash
ros2 run rqt_topic rqt_topic
```

### Service Call Testing
To interact with available services and test calls:
```bash
ros2 run rqt_service_caller rqt_service_caller
```

