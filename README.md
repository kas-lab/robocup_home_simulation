# KAS Lab Robocup@Home Simulation
This is the ROS2 Humble Ignition Fortress branch of the TU Delft KAS Lab's Robocup@Home simulation git repository.
This simulation currently supports two robots: the Mirte Master, and Albert.

## Installation
### Docker Installation
We currently offer a dockerized version of the Mirte simulation specifically for use on Ubuntu, the steps to use it are as follows:

Pull the latest version of the docker image
```Bash
docker pull ghcr.io/kas-lab/robocup_home_simulation:fortress
```

Allow docker to use a GUI
```Bash
xhost +
```

Run a container of the image
```Bash
docker run --name kaslab_robocup -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro ghcr.io/kas-lab/robocup_home_simulation:fortress
```

Now you can proceed to the Run Simulation instructions below to try things out, you can open multiple terminals of the container using the following commmand:
```Bash
docker exec -it kaslab_robocup bash
```

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

To just launch the ignition simulation use the following command:
```Bash
ros2 launch robocup_home_simulation mirte_robocup.launch.py
```

Then, you can use this command for Nav2 + Rviz:
```Bash
ros2 launch mirte_navigation robot_navigation.launch.py
```

And this command for MoveIt2 + Rviz:
```Bash
ros2 launch mirte_moveit_config mirte_moveit.launch.py
```
