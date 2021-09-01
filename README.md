# Graph-based-Planning-Algorithms
This repository contains the implementations of classiacl graph based planning algorithms.

## Breadth First Search Algorithm
This project implements the generalized solver for a point robot using the Breadth
First Search algorithm. It has a Node class having the attributes and methods useful
for executing the BFS algorithm. Also, it keeps track of the parent node for
backtracking after finding the goal state for the maze. It uses the deque class from
collections module as the data structure to optimize the enqueue and dequeue
operations. NOTE: The obstacle space for the robot is defined using the half
plane equations and it has 8 actions i.e up, down, left, right, upleft, upright,
downleft and downright. For the implementation video checkout [this link](https://drive.google.com/file/d/1KIGiUc6lRY8RuYK_3aIm1XiIpMVz_Fxm/view?usp=sharing).

### Map Dimensions
![BFS](Breadth First Search/Map-updated dimentions.png)
 
### Pygame Simulation 
![BFS](Breadth First Search/bfs.gif)


## Dijkstra's Algorithm
This project implements the generalized solver for a point robot using Dijkstra's
algorithm. It has a Node class having the attributes and methods useful for executing
Dijkstra's algorithm. Also, it keeps track of the parent node for backtracking after
finding the goal state for the maze.

### Map Dimensions
![DJ](Astar/map.png)
 
### Pygame Simulation 
![DJ](Graph-based-Planning-Algorithms/Dijkstra's Search/dj.gif)


## Astar Algorithm 
This project implements the generalized solver for a point robot using Astar  
algorithm. It has a Node class having the attributes and methods useful for executing
the algorithm. Also, it keeps track of the parent node for backtracking after
finding the goal state for the maze.

 
### Pygame Simulation 
![Astar](Astar/Astar.gif)


## Astar with Differential constraints and simulation in ROS Gazebo
This is an extension of the previous Astar implementation with simulation in ROS gazebo environment. 

### Map Dimensions
![DJ](Astar_implemenatation_in_ROS/map.png)

### Pygame Simulation 
![Astar](Astar_implemenatation_in_ROS/astar.gif)

### Gazebo Simulation 
![Astar in ros](Astar_implemenatation_in_ROS/ros1.gif)


### Dependencies
1) rospy
2) rospkg
3) tf
4) geometry_msgs
5) sensor_msgs


### How to run the code 
1) The package for running the simulation is named *velocity_publisher*.
2) Please run the below command to launch the turtlebot3 in the gazebo
environment:

```roslaunch velocity_publisher velocity_publisher.launch```

3) After launching the above file, please run the below command to make the
robot go to the goal position:

```rosrun velocity_publisher robot_control```

__________________________________________________________________________

4) If you want to run for a different test case run the following command (It will generate a shortest_path.txt file with the waypoint nodes for the robot
to follow.):

```rosrun velocity_publisher A_star```

5) Then you can follow steps 2 and 3 again to simulate that path (Note: Please
change the robot spawning position (x, y, yaw) in the *bringup.launch* file
according to the ROS envt. coordinates).








