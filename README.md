# jigsaw-game-robot

## Hardware
robot: Franka panda

camera: Realsense 400 series

## Enviroment set up
pre prepare

1.pc with ubuntu

2.[ros](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

3.Franka ros package

4.Realsense SDK, ros package and librealsense2

Init ROS Workspace first
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```

clone this repository to your work space
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/gesheng199832/jigsaw-game-robot.git
```

install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Build the project:
```sh
$ cd ~/catkin_ws
$ catkin build
```

Add following to your .bashrc file:
```
source ~/catkin_ws/devel/setup.bash
```
## Run game!
```sh
$ cd roslaunch porj5 main.launch
$ cd ~/catkin_ws/src/jigsaw-game-robot/porj5/scripts
$ python nn.py
$ python main.py
```
## project video [here](https://www.bilibili.com/video/av54739320)
