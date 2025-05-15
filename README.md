# KAS Lab Robocup@Home Simulation
This is the ROS2 Humble Ignition Fortress branch of the TU Delft KAS Lab's Robocup@Home simulation git repository.
This simulation currently supports two robots: the Mirte Master, and Albert.

## Installation

### Local Installation
Instructions for getting the general dependencies for this repository. Check the instructions below to get the extra dependencies required to run the Mirte or Albert robot.

Make a workspace and clone this repository:
```Bash
mkdir -p ~/kaslab_robocup_ws/src
cd ~/kaslab_robocup_ws/src
git clone -b fortress https://github.com/kas-lab/robocup_home_simulation.git
```

Clone the common package dependencies:
```Bash
cd ~/kaslab_robocup_ws/
vcs import src < src/robocup_home_simulation/general_dependencies.repos
```

Install the remaining external dependencies:
```Bash
cd ~/kaslab_robocup_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:
```Bash
cd ~/kaslab_robocup_ws/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

***Mirte***

Get Mirte deps:
```Bash
cd ~/kaslab_robocup_ws/
vcs import src < src/robocup_home_simulation/mirte_dependencies.repos
```

Install deps:
```Bash
cd ~/kaslab_robocup_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Ignore the mirte_telemetrix package:
```Bash
cd ~/kaslab_robocup_ws/
touch src/mirte-ros-packages/mirte_telemetrix_cpp/COLCON_IGNORE
```

Build workspace:
```Bash
cd ~/kaslab_robocup_ws/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

***Albert***

Instructions to build the albert robot pkg (this process will be improved when releasing the albert pkg)

Clone albert repo to the workspace:
```Bash
cd ~/kaslab_robocup_ws/src
git clone git@github.com:tud-airlab-ros2/albert_ros2_package.git -b mohamed
```

Install franka deps:
```Bash
sudo apt install libpoco-dev
cd somewhereelse
git clone https://github.com/frankaemika/libfranka.git --recursive
cd libfranka
git checkout 0.13.2
git submodule update --init --recursive
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
sudo make install
```

## Run simulation

```Bash
source install/setup.bash
ros2 launch robocup_home_simulation simulation.launch.py
```

### Mirte

```Bash
ros2 launch robocup_home_simulation mirte_robocup.launch.py
```
