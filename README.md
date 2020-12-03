# Local_Octomap_Server
## Requirements :  
### Octomap:  
- sudo apt-get install ros-noetic-octomap*
- cd catkin_ws/src
- git clone https://github.com/OctoMap/octomap_mapping.git 
- cd ..
- catkin_make
## quick install :
- cd catkin_ws/src
- git clone https://github.com/tjj1998/local_octomap_server.git
- cd ..
- catkin_make
## main features：
save the obstacle pointclouds surrounding the robot with local octomap
## params :
- **pointcloud** (std::string, default: "/pointcloud")  
Description: topic name of the input pointcloud 
- **leaf_size** (double, default: "0.05")  
Description: leaf size of the octomap tree
- **max_range** (double, default: "3.0")  
Description: max distance between the camera and the target points 
