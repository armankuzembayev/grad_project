# grad_project


## Instructions

Clone a repository to your workspace/src folder:
```
git clone https://github.com/armankuzembayev/jb_intern_tasks.git 
```
In terminal go to your workspace folder:
```
roscore
```
In other terminal window:
```
catkin_make
source devel/setup.bash
```

### Usage
In other terminal window:
```
webots
```
In webots choose the world from worlds/labirint.wbt
In other terminal window:
```
source devel/setup.bash
rosrun grad_project robot 
```
In other terminal window:
```
source devel/setup.bash
roslaunch grad_project robot.launch 
```
In other terminal window:
```
source devel/setup.bash
rosrun rviz rviz
```
