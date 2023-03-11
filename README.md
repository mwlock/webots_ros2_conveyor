# Conveyor

This package provides a ROS2 interface example for controlling a conveyor belt in Webots.

<img src="belt_gif.gif" width="100%">

## Topics
 
 - ```<robot_name>/cmd_vel``` Command velocity of the conveyor belt motor using a ```geometry_msgs/Twist``` message.
- ```/<cambot>/camera``` Camera image topic using a ```sensor_msgs/Image```message.
- ```/<cambot>/camera_info``` Camera info topic using a ```sensor_msgs/CameraInfo``` message.