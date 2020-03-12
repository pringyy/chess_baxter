cd ../../

catkin_make
xfce4-terminal -e "bash devel/setup.bash && roslaunch baxter_gazebo baxter_world.launch";
xfce4-terminal -e "bash devel/setup.bash && rosrun baxter_tools enable_robot.py -e";
xfce4-terminal -e "rosrun baxter_interface joint_trajectory_action_server.py";
xfce4-terminal -e "bash devel/setup.bash && roslaunch chess_baxter spawn_chessboard.py";
xfce4-terminal -e "bash devel/setup.bash && roslaunch chess_baxter gazebo2tfframe.py";
xfce4-terminal -e "bash devel/setup.bash && roslaunch chess_baxter pick_and_plave_moveit.py";