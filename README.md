# robocup_home_simulation
Robocup@Home simulation


## Albert

Instructions to build the albert robot pkg (this process will be improved when releasing the albert pkg)

Clone albert repo to the workspace:
```Bash
cd ~robocup_ws/src
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

## Build

Get repo:
```Bash
mkdir -p ~/robocup_ws/src
cd ~/robocup_ws/
git clone git@github.com:kas-lab/robocup_home_simulation.git
```

Get github deps:
```Bash
cd ~/robocup_ws/
vcs import src < robocup_home_simulation/simulation.rosinstall
```

Install deps:
```Bash
cd ~/robocup_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build package:
```Bash
cd ~/robocup_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Run simulation

```Bash
source install/setup.bash
ros2 launch robocup_home_simulation simulation.launch.py
```
