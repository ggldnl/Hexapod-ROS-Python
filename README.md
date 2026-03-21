# ROS2 Python Hexapod Controller Node

This repository contains a ROS2 python node wrapping the [hexapod controller](https://github.com/ggldnl/Hexapod-Controller.git).

## 🛠️ Setup

### Create an environment

Building this node will require having the [python controller](https://github.com/ggldnl/Hexapod-Controller.git).
The controller is managed as a git submodule and automatically downloaded.
When `colcon build` installs the node it only knows about the package, 
any external code from a submodule won't automatically be on the python path 
at runtime unless we explicitly make it available.

To solve this, we will work inside an empty environment.

```bash
conda create --name hexapod_controller_node
...
conda activate hexapod_controller_node
```

### Clone the repo

Clone the repo. For simplicity, I will assume the ROS workspace is in the `home` folder. 
A ROS best practice is to put any packages in the workspace into the `src` directory.

```bash
cd ~/ros_ws/src  # use your actual ROS workspace
git clone https://github.com/ggldnl/Hexapod-ROS-Python.git
```

Install the controller (`hexapod_controller_node` environment should be active):

```bash
pip install -e ./Hexapod-ROS-Python/Hexapod-Controller
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
ros2 run hexapod_controller hexapod_controller --ros-args -p port:=/dev/ttyAMA0 -p config_path:=/path/to/node/Hexapod-Controller/config/config.yml
```

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.