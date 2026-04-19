# ROS2 Python Hexapod Controller Node

This repository contains a ROS2 python node wrapping the [hexapod controller](https://github.com/ggldnl/Hexapod-Controller.git).

For a complete overview of the project refer to the [main Hexapod repository](https://github.com/ggldnl/Hexapod.git).

## 🛠️ Setup

Prerequisite: having ROS2 installed. I am running Ubuntu Server 24.04 on the RPI. The recommended stable distribution is [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
which was specifically designed to target Ubuntu 24.04 as its primary platform.

> Note:️ on a fresh Ubuntu install, following the official ROS2 guide, 
> I had problems installing `ros-dev-tools` due to a library version mismatch.
> I solved them with:
> ```bash
> sudo apt install libbz2-1.0=1.0.8-5.1 -y --allow-downgrades
> sudo apt install bzip2 -y
> sudo apt update && sudo apt install ros-dev-tools
> ```

### Install dependencies

```bash
sudo apt install ros-<your-distro>-tf-transformations
```

### Clone the repo

Clone the repo. For simplicity, I will assume the ROS workspace is in the `home` folder. 
A ROS best practice is to put any packages in the workspace into the `src` directory.

```bash
cd ~/ros_ws/src  # use your actual ROS workspace
git clone https://github.com/ggldnl/Hexapod-ROS-Python.git
```

### Install the controller

Building this node will require the [python controller](https://github.com/ggldnl/Hexapod-Controller.git) to be installed as a python package.
When `colcon build` installs the node it only knows about packaged code installed for system python:

```bash
pip install --break-system-packages git+https://github.com/ggldnl/Hexapod-Controller.git
```

### Build the package 

```bash
cd ~/ros_ws
colcon build --packages-select hexapod_controller
source install/setup.bash
```

### Verify it's working

Run the node:

```bash
ros2 run hexapod_controller hexapod_controller
```

In a second terminal, listen to the topic:

```bash
source ~/ros_ws/install/setup.bash
ros2 topic echo /robot_state
```

You should see the status of the robot (e.g. `IDLE`) printed every second on the terminal.

## 🚀 Delpoy

Run the node:

```bash
ros2 run hexapod_controller hexapod_controller
```

ROS2 parameters can be overridden at launch time without touching the code:

```bash
ros2 run hexapod_controller hexapod_controller --ros-args -p port:=/dev/ttyAMA0 -p config_path:=/path/to/node/Hexapod-Controller/config/config.yml
```

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.