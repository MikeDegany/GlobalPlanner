# GlobalPlanner

This repo is part of a project for motion planning for autonmous vehicles in dynamic environments. The project uses Hierarchical approach in order to make the best planner considering the designer criteria.
A hierarchical approach combines different motion planning techniques in two stages: Global Planning, and Local Planning. 

The implementation of a global planner using __*A** *algorithm*__ as a plugin for ROS-Navigation stack.
Take a look at the following address to follow the instruction on how to make a plugin for ROS Navigation stack:
 <a target="_blank" href="http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS">Writing A Global Path Planner As Plugin in ROS</a>.
 
This repository is part of the project for implementing global planner using A* algorithm and Gradient Descent method. 
There are two classes: Astar, and GradientDescent. As is obvious, the first one is to implement A* algorithm and the latter one is designed to use gradient descent method.
First, the value of the cells(grid squares or pixels of the map) of a part of the map will be calculated using A* method. A* is an informed graph based search method. Therefore, it only process the part of the map that the path is more likely to be in that. In order to do so, it uses a heuristic which has been defined as Eucleaden Distance in this project. Then, the quantifyed map will be used to calculate the interplolated gradient descent, which will be used to extract the path.


Further descriptions will be added soon..
