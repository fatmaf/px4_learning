*Assumption* _ROSMonitoring is used to generate the monitor package which is not part of this repo. The config files can be found in ROSMonitoringConfigs_
* Install PX4 
* ROS workspace with PX4
* Download the following packages:
  * https://github.com/fatmaf/px4_offboard_py
  * https://github.com/fatmaf/px4_rosmondemo_monitor
* Create a new catkin workspace and put the above two in the source folder (src)
* Catkin make 
* Download ROSMonitoring - copy the file uav_property1.py from rosmondemo/ROSMonitoringConfigs to the oracle folder 

### Launching the PX4 Simulation 

`cd <PX4-Autopilot_clone>`

`source ~/catkin_ws/devel/setup.bash    # (optional)`

`source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)`

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo/sitl_gazebo`

#### For ROSMonitoring with no user input 

* where it is true `roslaunch rosmondemo start_offb_locs_instrumented.launch` (dont forget to press enter because I wanted to wait)
* where it is  not true `roslaunch rosmondemo start_offb_locs_bad_instrumented.launch`


##### Starting the monitoring stuff 

``./oracle.py  --online --property uav_property1 --port 8080 --dense``


`` roslaunch monitor run.launch ``


#### For ROSMonitoring with user input 

get the QGroundControl App from the PX4 website 
* run `roslaunch rosmondemo monitor_verdict_visual_instrumented.launch`
* start the app
* start the monitor `` roslaunch monitor run.launch ``
* ``./oracle.py  --online --property uav_property_simplified --port 8080 --dense``