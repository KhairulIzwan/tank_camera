# tank_camera

## Notes
> ### Creating a catkin Package
>> 1. cd ~/catkin_ws/src
>> 2. catkin_create_pkg tank_camera roscpp rospy std_msgs message_generation


> ### Building a catkin workspace and sourcing the setup file
>> 1. cd ~/catkin_ws
>> 2. catkin_make
>> 3. . ~/catkin_ws/devel/setup.bash

## Clone and build the package
> ### tank_camera
>> 1. cd ~/catkin_ws/src
>> 2. git clone https://github.com/KhairulIzwan/tank_camera.git
>> 3. cd ~/catkin_ws
>> 4. catkin_make
>> 5. . ~/catkin_ws/devel/setup.bash

## Required Package
> ### cv_camera
>> 1. cd ~/catkin_ws/src
>> 2. git clone https://github.com/OTL/cv_camera.git
>> 3. cd ~/catkin_ws
>> 4. catkin_make
>> 5. . ~/catkin_ws/devel/setup.bash

# Usage

## Preview Image
> ### Previewing image (image_raw --> opencv)
>> Terminal # 1
>>> roscore

>> Terminal # 2
>>> rosrun cv_camera cv_camera_node _device:=/dev/video0 _image_width:=320 _image_height:=240

>> Terminal # 3
>>> rosrun tank_camera camera_preview.py

## HSV Color Range Trackbar
> ### Color range apps
>> Terminal # 1
>>> roscore

>> Terminal # 2
>>> rosrun cv_camera cv_camera_node _device:=/dev/video0 _image_width:=320 _image_height:=240

>> Terminal # 3
>>> rosrun tank_camera color_range_trackbar.py
