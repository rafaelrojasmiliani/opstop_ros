# Minimum-Time Path-Consistent Stopping Library for Collaborative Robots in ROS

Implementation of the [opstop](https://github.com/rafaelrojasmiliani/opstop_cpp) library in ROS.

Provides a relay node `follow_joint_trajectory_wrapper` which translates between a FollowJointGSpline action and a FollowJointTrajectory action.
On a goal request, This nodes converts the GSpline into a FollowJointTrajectory message and forwards the desired trajectory.
When FollowJointGSpline action is cancelled or prehempted, it compuntes the minium time path-consistent stop, and sends a new FollowJointTrajectory with the optimal stop trajectory.
This node is intened to interact with `ros_control`.


- MoveIt implementation [here](https://github.com/rafaelrojasmiliani/gsplines_moveit)

# Example
Launch the relay node
```bash
$ rosrun
```

# Installation

## In ubuntu with ROS

1. Install dependencies
```
apt-get update
apt-get install ros-${ROS_DISTRO}-ifopt python3-matplotlib libgtest-dev cmake libeigen3-dev coinor-libipopt-dev
```
2. Install the GSplines
```
source /opt/ros/${ROS_DISTRO}/setup.bash && git clone --recursive https://github.com/rafaelrojasmiliani/gsplines_cpp.git ~/gsplines && cd ~/gsplines && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=/usr && make && make install \
   && rm -rf ~/gsplines
```

3. Download this repo to your workspace and `catkin build`, or install
