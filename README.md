# GlobalPlanner

One of the main advantages of ROS is that programs can be modular. This repo is part of a project for motion planning for autonmous vehicles in dynamic environments. The project uses Hierarchical approach in order to make the best planner considering the design criteria.
A hierarchical approach combines different motion planning techniques in two stages: Global Planning, and Local Planning, and this repo is designed to solve the Global planning problem.
The implementation of a global planner using __*A** *algorithm*__ as a plugin for ROS-Navigation stack.
Take a look at the following address to follow the instruction on how to make a plugin for ROS Navigation stack:
 <a target="_blank" href="http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS">Writing A Global Path Planner As Plugin in ROS</a>.
 
It does not matter what kind of local planner you are going to use. The following will explain how to use this planner with the well known __turtlebot__ package. 


1. Considering that you have installed ROS on your system, you can install all the packages about turtlebot if you want or if you are not well familiar about ROS using the following command. 
```bash 
sudo apt-get install ros-[ROS distribution]-turtlebot* 
```
In any case, we are going to work with ```turtlebot-navigation``` package.
   
2. roscd to turtlebot_navigation package. Go to the lauch/includes directory and add following line in the file move_base.launch.xml as the super user (```sudo```) : ```<param name="base_global_planner" value="astar/Astar" />```
     
     
3. Launch **stage** and **rviz** with turtlebot:
    
   roslaunch turtlebot_stage turtlebot_in_stage.launch
   
4. Now click on “2D nav goal” button (at the top) in rviz and choose a goal location.

---


 This repository is part of the project for implementing global planner using A* algorithm and Gradient Descent method. 
There are two classes: ‍‍‍```Astar```, and ```GradientDescent```. As is obvious, the first one is to implement A* algorithm and the latter one is designed to reconstruct the path with gradient descent method.
The environment of the robot is represented as a Grid. The grid consists of many squares which we call them Cells in this project. Every Cell has an index and a value. First, the value of the cells(grid squares or pixels of the map) of a part of the map will be calculated using A* method. A* is an informed graph based search method. Therefore, it only process the part of the map that the path is more likely to be in that. In order to do so, it uses a heuristic which has been defined as Eucleaden Distance in this project. Then, the quantifyed map will be used to calculate the interpolated gradient descent, which will be used to reconstruct the path.

---




Further descriptions will be added soon..
