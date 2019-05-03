# Torso detection package through LaserScan 2D points.

 - This package listens to the `/scan_ubg` laser scan message.
 - Tested on *ROS Kinetic* and *Python2.7*
 - Torso markers publishes on `/TorsoMarker` topic.

## Usage:

 - Clone this repo.
 - `catkin_make`
 - `rosrun agn_torso_detection agn_torso_detection.py `

### TODO:
 - Refactoring (I wrote this node, when I was a newbie in Python.)
 - Make it as parametric-able by ros-param.
