
# MDP Group18: Automatic Cow Barn Robot Cleaner
This is the general README.md of our project, packages we created include their own README.md themselves as wel.
## Table of Contents
1. [Introduction](#introduction)
2. [Code Installation Steps](#code-installation-steps)
3. [Steps to Deploy the Robot](#steps-to-deploy-the-robot)
4. [Usage](#usage)
5. [Contributors](#contributors)
6. [License](#license)

## Introduction
Our solution focuses on providing the farmer with a solution that is intuitive, informative, and efficient. Our solution is easily managed by a user-friendly web management platform where the farmer can easily schedule cleaning sessions, get real-time progress updates from the robots, and easily control the robots whenever required.

The robotic platform itself is programmed to operate in the environment efficiently and robustly. Efficiency gains are enabled by a Global Mission Planner that analyzes the farmer's unique barn as well as the number of robots available to determine the most efficient path for each robot to follow. These efficiency gains are furthered by the Local Mission Planner that optimally adds the detected faeces to the path. The Local Mission Planner also ensures that the robot is robust to environmental challenges. It can dynamically adapt around unreachable areas, and ensure that the robot never runs out of battery in the middle of the barn.

The robot also uses state-of-the-art ML models (EfficientDet) to detect faeces and tracks them using the Exponentially Weighted Moving Average (EWMA) algorithm. This allows the robot to not only ensure that these faeces are being cleaned, but also enables the robot to plan ahead and thereby increase efficiency.
## Code Installation Steps
Provide step-by-step instructions to set up the development environment and install the necessary code.

### Prerequisites
- Ensure you have a ROS workspace set up. For this project, it is expected that you have a workspace named `mirte_ws`.
- Ensure you are connected to the robot via SSH over the same network connected to the internet.
- For general instructions on charging and booting the robot, please refer to: [https://docs.mirte.org/](https://docs.mirte.org/)

### Installation
1. **Clone the Repository**: Pull the repository into the `src` folder of your workspace.
    ```bash
    cd ~/mirte_ws/src
    git clone https://github.com/yourusername/your-repo.git
    ```

2. **Build the Workspace**: Navigate to the root of your workspace and build it.
    ```bash
    cd ~/mirte_ws
    catkin_make
    ```

3. **Source the Setup File**: Source the setup file to ensure your environment is configured correctly.
    ```bash
    source devel/setup.bash
    ```

4. **Install Dependencies**: Use `rosdep` to install all necessary dependencies.
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

5. **Automatic Node Launching**: The shell scripts that launch all necessary nodes will be set to execute automatically whenever the robot boots up.

### Summary of Commands
```bash
# Navigate to the src folder of your workspace
cd ~/mirte_ws/src

# Clone the repository
git clone https://github.com/yourusername/your-repo.git

# Navigate to the root of your workspace
cd ~/mirte_ws

# Build the workspace
catkin_make

# Source the setup file
source devel/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Shell Script for Automatic Node Launching

Save the following script as `startup.sh` in your home directory:

```bash
#!/bin/bash

# To make this script run at startup of the robot,
# add the following line to your rc.local file :
# '/home/yourusername/mirte_ws/startup.sh'

# Get the IP address of the machine
IP=$(hostname -I | awk '{print $1}')

# Set the ROS IP
export ROS_IP=$IP

# Set the ROS Master URI
export ROS_MASTER_URI=http://$IP:11311

# Source your ROS workspace
source ~/mirte_ws/devel/setup.bash

# Launch mandatory nodes for the robot
roslaunch launch_group18 perception.launch
roslaunch global_mission_planner global_mission_planner.launch
roslaunch local_mission_planner local_mission_planner.launch
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch launch_group18 hri_communication.launch
```

To make this script run at startup, add the following line to your `rc.local` file:

```bash
/home/yourusername/mirte_ws/startup.sh
```

## Steps to Deploy the Robot

1. **Charge the Robot**: Ensure the robot is fully charged. For general instructions on charging and booting the robot, please refer to: [https://docs.mirte.org/](https://docs.mirte.org/)
   
2. **Connect to Wireless Access Point**: Connect the robot to the wireless access point. Ensure that the barn has wireless network coverage. For connecting the robot to a wireless network, please refer to: [https://docs.mirte.org/doc/connect_to_mirte.html#wireless-acces-point](https://docs.mirte.org/doc/connect_to_mirte.html#wireless-acces-point)

3. **Start the Robot**: Power on the robot and wait for it to boot up (wait for the screen to turn on).

4. **Start the Robot Interface**: Open a web browser (on any device connected to the same network) and navigate to the robot's IP address, shown on the robot. This will open the robot's interface.

5. **Prepare the Barn**:
   - Remove all moving objects from the barn.
   - Place the robot anywhere inside the barn.

6. **Start the Mapping Process**:
   - Use the provided interface to start the mapping process.
   - Manually control the robot using the interface to ensure all areas are mapped.
   - Ensure all static objects are included by checking the map within the interface.

7. **Stop the Mapping Process**:
   - Once mapping is complete, stop the mapping process using the interface.
   - A map will be saved automatically.

8. **Initiate Cleaning**:
   - The robot is now ready for cleaning.
   - Manually initiate a cleaning session using the button in the web interface to start a session or schedule sessions through the web interface.

## Contributors
GROUP 18:
- **Siqi Pei** - [S.Pei@student.tudelft.nl](mailto:S.Pei@student.tudelft.nl)
- **Maurits Heemskerk** - [M.J.Heemskerk@student.tudelft.nl](mailto:M.J.Heemskerk@student.tudelft.nl)
- **Shantnav Agarwal** - [S.Agarwal-19@student.tudelft.nl](mailto:S.Agarwal-19@student.tudelft.nl)
- **Quentin Missinne** - [Q.Missinne@student.tudelft.nl](mailto:Q.Missinne@student.tudelft.nl)
- **Tijn Vennink** - [T.Vennink@student.tudelft.nl](mailto:T.Vennink@student.tudelft.nl)


Course RO47007, Msc. Robotics, Tu Delft.
