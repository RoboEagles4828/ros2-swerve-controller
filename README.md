# 2023RobotROS
ROS Code for the 2023 Robot

## Requirements
-------
- Ubuntu 20.04
- RTX Enabled GPU

# Workstation Setup steps

### 1. ROS2 Setup
- **Install ROS2 Galactic** \
`sudo ./scripts/install-ros2-galactic.sh`

- **Install ROS VS Code Extension** \
Go to extensions and search for ROS and click install

- **Install Workspace Dependencies** \
Press F1, and run this command: \
`ROS: Install ROS Dependencies for this workspace using rosdep`

- **Build the packages** \
Shortcut: `ctrl + shift + b` \
or \
Terminal: `colcon build --symlink-install`

- **Done with ROS2 Setup**

### 2. Isaac Sim Setup

- **Install Nvidia Drivers** \
`sudo apt-get install nvidia-driver-515`

- **Download Omniverse Launcher** \
https://www.nvidia.com/en-us/omniverse/download/

- **Run Omniverse** \
`chmod +x omniverse-launcher-linux.AppImage` \
`./omniverse-launcher-linux.AppImage`

- **Install Cache and Isaac Sim from the exchange** \
Wait until both are downloaded and installed.

- **Make ROS2 the default bridge** \
`sudo ./scripts/use-isaac-ros2-bridge.sh`

- **Launch Isaac Sim** \
Initial load can take a while

- **Open the extension manager** \
`Window -> Extensions`

- **Import the Swerve Bot Extension** \
Hit the gear icon near the search bar. \
Inside Extension Search Paths, Hit the + icon and put the path \
`<path-to-repo>/2023RobotROS/isaac`

- **Enable the Extension** \
Search for the Swerve Bot extension. \
Click on the extension. \
Click on the enable toggle switch and Autoload checkbox

- **DONE with Isaac Sim Setup**


# Running Swerve Bot

1. Connect an xbox controller  
2. Hit load on the *Import URDF* extension window
3. Press Play on the left hand side
4. From this repo directory run \
`source install/setup.bash` \
`ros2 launch swerve_description isaac.launch.py`
5. Use the controller to move the robot around