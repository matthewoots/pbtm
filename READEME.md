# PX4 B-spline Trajectory Manager

## Introduction
`pbtm` serves as a mission manager for PX4 related systems using Mavros, it employs the use of `libbspline` package which creates Bspline trajectories for the agent. Running on 2 threads, one is handling the `drone_timer` and the other is for callbacks.

- `sample_sitl.launch` shows a sample of running with `PX4-SITL` on gazebo with Rviz

![Alt Text](pbtm_x2.gif)

---
To give a waypoint/s this module utilizes `JointTrajectory` message, where `JointTrajectory` branches includes `poses` vector and `header` 

- poses[] = `JointTrajectoryPoint` http://docs.ros.org/en/lunar/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html
    - positions [`x, y, z`]
    - time_from_start [`1.0, 2.0, 3.0, 4.0, 5.0`]
        - 1.0 = **Takeoff**
        - 2.0 = **Hover**
        - 3.0 = **Mission**
        - 4.0 = **Home**
        - 5.0 = **Land**

To add a command, you will have to add **Ros Duration** such as `rospy/roscpp.Duration(*command double*)` to the **first time_from_start** in the JointTrajectory poses vector

Examples can be found in python in `scripts`

### Dependencies
- libbspline (https://github.com/matthewoots/librrtserver.git)
- mavros (Follow the guide in https://docs.px4.io/v1.12/en/ros/mavros_installation.html)
- Eigen

### Setup 
For starters who do not know about ROS and/or PX4, or do not have a prior workspace, just run the commands below and all will be fine

---
#### **Setting up PX4**
```bash
# Start in home directory ~/
# For setting up PX4
# If you do not have PX4 on your local machine using ssh
git clone git@github.com:PX4/PX4-Autopilot.git 
# or for https
git clone https://github.com/PX4/PX4-Autopilot.git

# Checkout to stable version in tag 1.12.3
cd PX4-Autopilot
git checkout v1.12.3
git submodule update -- init --recursive

# For a fresh machine without any PX4, have not downloaded the PX4 dependencies for python etc
cd Tools/setup
./ubuntu.sh

# Compile without running and resolve issues or bugs
cd ~/PX4-Autopilot
make px4_sitl gazebo DONT_RUN=1
```

---
#### **Setting up ROS and package**
```bash
# Install mavros and geographic library if you have not
sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# make directory for catkin workspace if you do not have
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/matthewoots/pbtm.git --recurse-submodules
cd ..
catkin build pbtm

# If PX4-Autopilot is in ~/ directory
# copy setup.bash from repository and replace the one in devel
cd ~/catkin_ws/src/pbtm
cp setup.bash ../../devel/setup.bash
source devel/setup.bash
```
### Simple Run
1. Launch `roslaunch pbtm sample_sitl.launch`
2. You will see an rviz pop up, with the drone identified as `drone0` after EKF2 has initialized
3. `cd ~/catkin_ws/src/pbtm\scripts` and run `python send_target_takeoff.py` followed by `python send_target_random.py` after it has taken off

Upstream can be found in https://github.com/matthewoots/pbtm