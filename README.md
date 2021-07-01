# Local_Octomap_Server
## Requirements :
### Octomap:
- sudo apt-get install ros-noetic-octomap*
- git clone https://github.com/OctoMap/octomap_mapping.git and catkin_make
### MapServer:
- sudo apt-get install ros-noetic-map-server
## quick install :
- cd catkin_ws/src
- git clone https://gaoyichao.com:61698/third/localoctomap_server.git
- cd ..
- catkin_make
## main features：
save the obstacle pointclouds surrounding the robot with local octomap
## service :
- **ClearOctomap**  
Description: clear the Octree, the pointcloud and the gridmap  
req: cmd (bool, true)  
res: None  

- **SaveGridmap**  
Description: save the gridmap to specified path    
req: map_savepath(std::string, "/home/yourusername/map/")  
Description: the last path represents the girdmap name ,which is a directory 
res: state(bool, "true")  
Description: true if saved map success  

## params :
- **pointcloud** (std::string, default: "/pointcloud")  
Description: topic name of the input pointcloud  

- **base_frame** (std::string, default: "/map")  
Description: topic name of the fix frame  

- **leaf_size** (double, default: "0.05")  
Description: leaf size of the octomap tree  

- **max_range** (double, default: "3.0")  
Description: max distance between the camera and the target points  

- **max_rayRange** (double, default: "3.0")  
Description: max distance for the KeyRay

- **publish_gridmap** (bool, default: "true")  
Description: whether process gridmap

- **loadmap_path** (std::string, default: "map.yaml")  
Description: work when publish_gridmap is true, not load if value equals ""

- **incremental_update** (bool, default: "true")  
Description: work when publish_gridmap is true, whether update the gridmap incrementally