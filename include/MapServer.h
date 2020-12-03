#pragma once

#include "ros/console.h"
#include "ros/ros.h"
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <chrono>
#include <map>
#include <nav_msgs/Path.h>

typedef std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudPtr;
class MapServer
{
    public:
        typedef std::shared_ptr<MapServer> Ptr;
        MapServer(double leaf_size=0.02);
        void reset();
        void insertLocalPointCloud(PointCloudPtr local_pc, const Eigen::Vector3d cam, double range=3.0);
        void getOctomapMsg(octomap_msgs::Octomap& mymap_msg);
        void getPCLPointCloud(PointCloudPtr pc);
    private:
        octomap::OcTree* tree; // 参数为分辨率
};

