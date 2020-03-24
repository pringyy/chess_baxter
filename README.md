# Chess Baxter
This project was developed as a coursework task for the Robotics Foundations (H) course at the University of Glasgow. The aim was to develop a solution to enable the Baxter robot (https://en.wikipedia.org/wiki/Baxter_(robot)) to play chess using Python, ROS and associated technologies including Gazebo and RViz.

Baxter is a two-armed robot with an animated face. It is 3 feet tall and weighs 165 lbs without its pedestal; with its pedestal it is between 5'10" – 6'3" tall and weighs 306 lbs. It is used for simple industrial jobs such as loading, unloading, sorting, and handling of materials.

## Playing Chess
Clone the repository, run `catkin_make` and source your workspace.  
Run the following in separate terminals:
```
roslaunch baxter_gazebo baxter_world.launch
```

Terminal 1:
```
roslaunch baxter_gazebo baxter_world.launch
```
Terminal 2:
```
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
```
Terminal 3:
```
roslaunch baxter_moveit_config baxter_grippers.launch
```
Terminal 4:
```
rosrun chess_baxter spawn_chessboard.py
```
Terminal 5:
```
rosrun chess_baxter pick_and_plave_moveit.py
```
Terminal 6:
```
rosrun chess_baxter gazebo2tfframe.py
```

In order to delete the chessboard and its components, run the command:
```
rosrun chess_baxter delete_chessgame.py
```

## Video Demo
[Chess Baxter Demo](https://youtu.be/x4oP9nS4Y-I)
