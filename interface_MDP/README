# README

## Setup Instructions

### Dependencies

1. **Install ROS Packages**:
    ```bash
    sudo apt install ros-noetic-rosbridge-server
    ```

2. **Install Node.js and npm**:
    ```bash
    sudo apt update
    sudo apt install nodejs npm
    ```

3. **Install npm packages** (navigate to the interface directory first):
    ```bash
    cd group18/interface_MDP
    npm install
    ```

### Terminal 1: Starting the ROS Master

1. Source the workspace environment:
    ```bash
    source ./devel/setup.bash
    ```
2. Stop the `mirte-ros` service:
    ```bash
    sudo service mirte-ros stop
    ```
3. Set the ROS IP to your own IP address:
    ```bash
    export ROS_IP=145.94.141.169
    ```
4. Launch the minimal master:
    ```bash
    roslaunch mirte_bringup minimal_master.launch
    ```

### Terminal 2: Starting Rosbridge

1. Source the workspace environment:
    ```bash
    source ./devel/setup.bash
    ```
2. Source the ROS environment:
    ```bash
    source /opt/ros/noetic/setup.bash
    ```
3. Set the ROS IP to your own IP address:
    ```bash
    export ROS_IP=145.94.190.160
    ```
4. Set the ROS Master URI to the robot's IP address:
    ```bash
    export ROS_MASTER_URI=http://145.94.149.160:11311
    ```
5. Launch the Rosbridge WebSocket server:
    ```bash
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

### Terminal 3: Starting HRI Communication

1. Source the workspace environment:
    ```bash
    source ./devel/setup.bash
    ```
2. Set the ROS IP to your own IP address:
    ```bash
    export ROS_IP=145.94.141.169
    ```
3. Set the ROS Master URI to the mirte's IP address:
    ```bash
    export ROS_MASTER_URI=http://145.94.149.160:11311
    ```
4. Launch the HRI communication:
    ```bash
    roslaunch hri_communication hri_communication.launch
    ```

### Terminal 4: Starting the Interface

1. Source the ROS environment:
    ```bash
    source /opt/ros/noetic/setup.bash
    ```
2. Set the ROS IP to your own IP address:
    ```bash
    export ROS_IP=145.94.190.160
    ```
3. Set the ROS Master URI to the robot's IP address:
    ```bash
    export ROS_MASTER_URI=http://145.94.149.160:11311
    ```
4. Navigate to the interface directory:
    ```bash
    cd group18/interface_MDP
    ```
5. Start the server:
    ```bash
    node server.js
    ```

### Additional Step

- Press `F12` in your browser and **disable cache** for mapping.

