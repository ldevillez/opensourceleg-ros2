# OpenSource-Leg ROS2 Node

This repository provides a ROS2 node implementation for the [OpenSource-Leg (OSL)](https://opensourceleg.org/), an open-source robotic leg platform developed by the Neurobionics Lab. The node enables communication, control, and monitoring of the OSL hardware within a ROS2 ecosystem.

## Project Structure

```
osl_interface/
    ├── action/           # Custom ROS2 actions
    ├── msg/              # Custom ROS2 messages
    ├── srv/              # Custom ROS2 services
    ├── CMakeLists.txt    # Build configuration
    └── package.xml       # ROS2 package manifest
osl_node/
    ├── osl_node/         # Python ROS2 node implementation
    ├── resource/         # ROS2 resource files
    ├── test/             # Unit and style tests
    ├── setup.py          # Python package setup
    └── package.xml       # ROS2 package manifest
```

## Features
- ROS2 node for controlling and monitoring the OpenSource-Leg
- Custom messages, services, and actions for leg operation
- Modular and extensible design


## Getting Started

### Prerequisites
- [ROS2 Humble or later](https://docs.ros.org/en/kilted/Installation.html) (only tested on Rolling)
    - If you are not familliar with ROS2, it is suggested to follow the tutorials
- Python 3.11+
- OSL V2 hardware

### Installation
1. Install ubuntu on your raspberry pi
    - The [robot-ci](https://github.com/neurobionics/robot-ci) tool also support Ubuntu
2. Install ROS2: https://docs.ros.org/en/kilted/Installation.html
    - On your raspberry pi
    - On another computer if you want to control the OSL remotely
3. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros_ws/src
   git clone https://github.com/ldevillez/osl_ros2.git
   ```
4. Install dependencies:
   ```bash
   cd ~/ros_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
5. Build the workspace:
   ```bash
   colcon build
   ```
6. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Usage
To launch the OSL ROS2 node for the knee:
```bash
ros2 run osl_node knee
```
or for the ankle:
```bash 
ros2 run osl_node knee
```

## How to use
### OSL Node
The [`osl_node`](osl_node) directory contains the main ROS2 node implementation for the OpenSource-Leg.

The node is able to manage either a knee or an ankle:
```bash
ros2 run osl_node <leg_type>
```
Where `<leg_type>` can be either `knee` or `ankle`. It is possible to run both an ankle and a knee node simultaneously by launching two separate instances of the node with the respective leg types.

When the node is runnig it will publish: 
- Joint kinematics on the `<leg_type>/joint_kinematic` topic
- Motor information on the `<leg_type>/motor_info` topic

It will listen to the following topics:
- Position target on the `<leg_type>/position_control` topic
- Joint impedance on the `<leg_type>/impedance_control` topic

It will also provide the following services:
- `<leg_type>/set_interface_status`: To set the interface status
- `<leg_type>/set_motor_control`: To set the motor control mode

And the following actions:
- `<leg_type>/go_to`: To move the joint to specified positions
- `<leg_type>/home`: To home the joint

By default, the interface status is set to `INIT` and the control mode is set to `MANUAL`.
Most of the functionality are not accessible if the node interface status is not set to `ENABLED`.

In the `MANUAL` control mode, the joint can be homed and moved to specified positions using the provided actions.
In the `TRACKING` control mode, the joint will follow the position targets published on the `<leg_type>/position_control` or the impedance control commands published on the `<leg_type>/impedance_control` topic. It will used the last received command to control the joint.


### Interface
The [`osl_interface`](osl_interface) directory, list all the custom messages, services and actions used by the OSL ROS2 node.

#### Messages
- `ControlMode.msg`: Defines control modes for the leg (e.g., manual, tracking, unknown).
- `InterfaceStatus.msg`: Status information from the leg interface.
- `JointImpedance.msg`: Joint impedance data.
- `JointKinematics.msg`: Joint kinematics data.
- `MotorInfo.msg`: Information about the motors.

#### Services
- `SetInterfaceStatus.srv`: Service to set the interface status.
- `SetMotorControl.srv`: Service to set the motor control mode.

#### Actions
- `JointGoTo.action`: Action to move joint to specified positions.
- `JointHome.action`: Action to home the joint.

### Controlling the OSL Node Remotely

By default, ROS uses a multicast discovery mechanism to find nodes on the same network. If you are not able to use multicast, you can set up ROS2 to work in a unicast mode by specifying the domain ID and the discovery server. More infos are available in the [ROS2 documentation](https://docs.ros.org/en/kilted/Tutorials/Advanced/Discovery-Server/Discovery-Server.html).

#### Quick setup
1. Check that the computer and the raspberry pi are on the same network and respond to each other (e.g., using ping).
2. Check that they use the same ROS_DOMAIN_ID (default is 0). You can set it using:
   ```bash
   export ROS_DOMAIN_ID=1  # or any other number
   ```
3. Based on this ID, check the the corresponding ports are open ([ROS2 port list](https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Domain-ID.html)).
4. On the local computer, create a discovery server:
    ```bash
    fastdds discovery --server-id 0
    ```
5. On all the machines, connect to the discovery server:
    ```bash
    ROS_DISCOVERY_SERVER=<IP SERVER>:11811
    ```
6. Now all the nodes should be able to communicate together.
7. CLI tools may require to also set another environment variable:
    ```bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=<FULL_PATH_TO_FILE>/super_client_configuration_file.xml
    ```
    You can find an example of configuration file [here](https://docs.ros.org/en/rolling/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#daemon-s-related-tools)

    Note: In the documentation, it uses the `FASTDDS_DEFAULT_PROFILES_FILE` variable, but in ROS2 Rolling and in my configuration it did not work. Using `FASTRTPS_DEFAULT_PROFILES_FILE` worked as expected.
    8. Restart the ros2 daemon:
    ```bash
    ros2 daemon stop
    ros2 daemon start
    ```




## Documentation
- [OpenSource-Leg Website](https://opensourceleg.org/)
- [OpenSource-Leg GitHub](https://github.com/neurobionics/opensourceleg)

## Contributing
Contributions are welcome! Please open issues or pull requests for improvements, bug fixes, or new features.

## License
This project is licensed under the Apache License. See the `LICENSE` file for details.

## Acknowledgements
- [Neurobionics Lab](https://neurobionics.umn.edu/)
- ROS2 Community

---

<p align="center">
  <img src="https://raw.githubusercontent.com/opensourceleg/opensourceleg.github.io/refs/heads/main/public/logo/osl-vertical.svg" alt="OpenSource-Leg" width="300"/>
</p>
