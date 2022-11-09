# Launching the PX4 Simulation 

`cd <PX4-Autopilot_clone>`

`source ~/catkin_ws/devel/setup.bash    # (optional)`
`source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`
`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)`
`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo/sitl_gazebo`



## Starting the monitoring stuff 

``./oracle.py  --online --property uav_property1 --port 8080 --dense``
`` roslaunch monitor run.launch ``
````