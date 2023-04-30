# path-smoothing-ros
## Overview
A cubic spline interpolator for path smoothing. Compatible with ROS.

### License

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone git@github.com:Wrybill-Robotics/path_smoothing_ros.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage
    roslaunch path_smoothing_ros path_smoothing.launch
## Launch files

* **path_smoothing.launch:**  generates a smooth path from input waypoints

## Nodes

### path_from_waypoints

    Loads a geometry_msgs/PoseWithCovarianceStamped and appends to path.

#### Subscribed Topics

* **`/input`** ([geometry_msgs/PoseWithCovarianceStamped])
    input topic to generate path points
#### Published Topics
* **`/initial_path`** ([nav_msgs/Path])
    simple straight line path between input poses    
#### Parameters


### path_smoother

    Loads a nav_msgs/Path and performes cubic spline interpolation to smooth the path

#### Subscribed Topics

* **`/initial_path`** ([nav_msgs/Path])
    Simple straight line path to perform smoothing in

#### Published Topics
* **`/smoothed_path`** ([nav_msgs/Path])
    Smoothed path

* **`/initial_pose`** ([geometry_msgs/PoseStamped])
    pose for start of smoothed path

* **`/final_pose`** ([geometry_msgs/PoseStamped])
    pose for end of smoothed path
#### Parameters
* **`points_per_unit`** ([Double])
    number of points to generate between input points
* **`skip_points`** ([Int])

* **`use_end_conditions`** ([Boolean])

* **`use_middle_conditions`** ([Boolean])











### References
- J. Hilgert, K. Hirsch, T. Bertram and M. Hiller, "Emergency path planning for autonomous vehicles using elastic band theory," Proceedings 2003 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM 2003), 2003, pp. 1390-1395 vol.2.
doi: 10.1109/AIM.2003.1225546
