# ROS2 & Gazebo Workspace

## Usage
### Build Workspace
```shell
# change into workspace directory
cd dev_ws

# install python dependencies: requires python 3.8
pip install -r 

# install ros dependencies: requires rosdep
rosdep install -i --from-path src --rosdistro <ros_distro> -y

# build workspace with colcon
colcon build --symlink-install
```
*Note: Please do not launch workspace in the same terminal as where it was built. I.e., open a new terminal tab/window for the next step.*

### Launch Simple Car Model
```shell
ros2 launch my_pkg simple_car.launch.py
```

### Launch Teleop Twist Keyboard
A Teleop Twist Keyboard can be used to control the simple car model.
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
