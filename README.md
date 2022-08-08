# SofAr Assignment
### Candidates: [Salvatore D'Ippolito](https://github.com/Salvo-Dippolito), [Davide Parisi](https://github.com/dpareasy), [Sara Sgambato](https://github.com/sarasgambato), [Ankur Kohli](https://github.com/)

The goal of this assignment is to explore the tools of Nav2, in particular the use of a costmap filter to establish keepout zones. A 3D model of the mobile 
robot TIAGo from PAL RObotics has been modeled to run in a custom 3D world built using Webots simulator. A 2D map of this 3D world is passed to Nav2 
through the use of the Map Server node. The keepout zones are also passed as 2D maps but they are processed by a different server called CostMap Filter. These servers are part of the Nav2 navigation system and have to be set up through the use of specific parameter files.

## Main contents of this Repository ##

* **worlds**: it's a folder containing different `.wbt` files of 3D environments;
* **params**: it's a folder containing different `.yaml` files for the setup of the costmap info server and for the keepout filter;
* **maps**: it's a folder containing all the `.yaml` files with their correspondent `.pgm` files of different 2D maps related to their `.wbt` files in the worlds folder;
* **launch**: it's folder containing the main launch file called `robot_launch.py` and three secondary launch files to load Nav2 navigation system;
* **sofar_assignment**: it's a folder containing the scripts of the node used to control TIAGo's behavior;
* **URDF**: it's a folder containing the information about TIAGo.

## How to run ##
 Clone this repository inside the src folder of your workspace:
 ```
 git clone https://github.com/dpareasy/sofar_assignment.git
 ```
 Build the package and source:
 ```
 colcon build
 
 source install/local_setup.bash
 ```
 Run the simulation:
 ```
 ros2 launch sofar_assignment robot_launch.py
 ```
 This is the command for the default simulation which launches the default world found in the Webots installation files and the relative costmap that we developed. To launch different costmap just add the following line as an argument of the previous command:
 ```
 mask:=src/sofar_assignment/maps/<name-of-the-mask>.yaml
 ```
 where in the `<name-of-the-mask>` field you should put the name of a `.yaml` file present in the maps folder. You can follow the same method to set also the world and its relative 2D map file:
```
ros2 launch sofar_assignment robot_launch.py map:=src/sofar_assignment/maps/emaro_map.yaml world:=src/sofar_assignment/worlds/emaro_lab.wbt mask:=src/sofar_assignment/maps/emaro_keepout_mask2.yaml params_file:=src/sofar_assignment/params/emaro_nav2_params.yaml keepout_params_file:=src/sofar_assignment/params/emaro_keepout_params_2.yaml
```
## Controlling the robot model ##
The robot can be controlled in two different ways:
* by using rviz user interface to give the robot a goal;
* by running the node `nav_to_goal.py` in another terminal:

  ```
  ros2 run sofar_assignment move_to_goal_exe
  ```
## References ##

* Steve Macenski reposotory for nav2: https://github.com/ros-planning/navigation2
* Tutorial for keepout zones: https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html

