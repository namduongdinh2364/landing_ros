# landing_ros
UAV Landing using the packages.
- [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros.git)
- [aruco_ros](https://github.com/pal-robotics/aruco_ros.git)
- [whycon](https://github.com/lrse/whycon.git)
- [mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers.git)
## Getting Started
### Install PX4 SITL(Only to Simulate)
Follow the instructions as shown in the [ROS with Gazebo Simulation PX4 Documentation](https://dev.px4.io/master/en/simulation/ros_interface.html)
To check if the necessary environment is setup correctly, you can run the gazebo SITL using the following command

```bash
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo
```
### Installing landing
```bash
mkdir -p ~/catkin_ws
cd ~/catkin_ws
catkin init
catkin config --merge-devel
```
###### Clone this repository

```bash
cd ~/catkin_ws/
git clone  https://github.com/namduongdinh2364/landing_ros.git
```
Now continue automatically download dependencies.
```bash
cd ~/catkin_ws/src
chmod +x ../git_clone.sh
../git_clone.sh
```
###### Add marker to gazebo
```bash
cd ~/catkin_ws
mv ./simulation/nested_tags_visual_marker <path PX4-Autopilot_clone>/Tools/sitl_gazebo/models
```
Add to file world (e.g <path PX4-Autopilot_clone>/Tools/sitl_gazebo/worlds/empty.world)
```
<world name="default">
  ...
  <include>
    <uri>model://nested_tags_visual_marker</uri>
    <pose>2 2 0.0 0.0 0.0 0.0</pose>
  </include>
  ...
</world>
```
###### Build all the packages:
```bash
cd ~/catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash
```
## Running the code
The following launch files enable UAV landing with the geometric controller:

``` bash
roslaunch geometric_controller sitl_trajectory_track_marker.launch
```
Another terminal runs:
``` bash
rosrun geometric_controller landing
```




