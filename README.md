# realsense_laserscan

[depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan) uses middle section of the depth image to create laserscan data. 

This repository creates laserscan from D435 depth map on a set height using T265 odometry. Set height is defined in horizontal camera position. 
Pitch angle is used to choose which part of the image is passed on to depthimage_to_laserscan algorithm. 
If pitch angle is too big, empty depth map is sent, resulting in no laserscan data output.

## Used repositories:
- [realsense_nodes_python](https://github.com/Michal-Bidzinski/realsense_nodes_python)
- [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan)
