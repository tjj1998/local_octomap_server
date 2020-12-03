#include "MapServer.h"
#include <glog/logging.h>

MapServer::MapServer(double leaf_size)
{
    tree = new octomap::OcTree(leaf_size);
}

void MapServer::reset() {
    tree->clear();
}

void MapServer::insertLocalPointCloud(PointCloudPtr local_pc, const Eigen::Vector3d cam, double range)
{ 
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(); it != tree->end_leafs(); it++)
    {
        octomap::point3d point = it.getCoordinate();
        Eigen::Vector3d p(point.x(),point.y(),point.z());
        double dist = (cam-p).norm();
        if(dist > range){
            tree->deleteNode(point,it.getDepth());
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    octomap::Pointcloud cloud_octo;
    for (auto p:local_pc->points)
        cloud_octo.push_back( p.x, p.y, p.z );

    tree->insertPointCloud( cloud_octo,octomap::point3d( cam(0), cam(1), cam(2) ) );
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    tree->updateInnerOccupancy();
    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    double track1= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    double track2= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    double track3= std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count();

    LOG(INFO) << "process1 costs:" <<track1*1000 <<"ms";
    LOG(INFO) << "process1 costs:" <<track2*1000 <<"ms";
    LOG(INFO) << "process1 costs:" <<track3*1000 <<"ms";
    LOG(INFO) << "insert localPointCloud success!"<< local_pc->points.size()<< " points in local_pc";
}

void MapServer::getOctomapMsg(octomap_msgs::Octomap& mymap_msg)
{
    octomap_msgs::fullMapToMsg(*tree, mymap_msg);
}

void MapServer::getPCLPointCloud(PointCloudPtr pc)
{
    pc->clear();
    int cnt = 0;
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs();it != end; ++it){
        pcl::PointXYZ point;
        point.x = it.getX();
        point.y = it.getY();
        point.z = it.getZ();
        if(tree->isNodeOccupied(*it)){
            pc->points.push_back(point);
        } 
        cnt++;
  }
  LOG(INFO)<<"octree size:"<<pc->points.size();
}
