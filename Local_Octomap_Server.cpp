#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <local_octomap_server/ClearOctomap.h>
#include <local_octomap_server/SaveGridmap.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <glog/logging.h>
#include "local_octomap_server/MapServer.h"


void Tf2Eigen(geometry_msgs::Transform const & tfmsg, Eigen::Matrix4f & transform) {
    Eigen::Matrix3f qua = Eigen::Quaternionf(tfmsg.rotation.w,
                                             tfmsg.rotation.x,
                                             tfmsg.rotation.y,
                                             tfmsg.rotation.z).toRotationMatrix();

    Eigen::Vector3f trs = Eigen::Vector3f(tfmsg.translation.x,
                                          tfmsg.translation.y,
                                          tfmsg.translation.z);
    transform = Eigen::Matrix4f::Identity();
    transform << qua, trs, Eigen::RowVector4f(0, 0, 0, 1);
}

class Local_Octomap_Server{
    public:
        Local_Octomap_Server(tf2_ros::Buffer & buf):tf_buf(buf){
          m_nh = new ros::NodeHandle("~");
          m_nh->param<std::string>("pointcloud", pc_topic_, "/pointcloud");
          m_nh->param<std::string>("base_frame", base_frame_, "/map");
          m_nh->param<double>("leaf_size", leaf_size_, 0.05);
          m_nh->param<double>("max_range", max_range_, 3.0);
          m_nh->param<double>("max_rayRange", max_rayRange_, 2.0);     
          m_nh->param<bool>("publish_gridmap", publish_gridmap_, true);
          m_nh->param<std::string>("loadmap_path", loadmap_path_, "");
          m_nh->param<bool>("incremental_update", incremental_update_, true);
          LOG(INFO) << "pointcloud:" << pc_topic_ ;
          LOG(INFO) << "base_frame:" << base_frame_ ;
          LOG(INFO) << "leaf_size:" << leaf_size_ ;
          LOG(INFO) << "max_range:" << max_range_ ;
          LOG(INFO) << "max_rayRange:" << max_rayRange_ ;
          LOG(INFO) << "publish_gridmap:" << publish_gridmap_ ? "true" : "false";
          LOG(INFO) << "loadmap_path:" << loadmap_path_ ;
          LOG(INFO) << "incremental_update:" << incremental_update_ ? "true" : "false";
          mapserver = MapServer::Ptr(new MapServer(leaf_size_,max_rayRange_,incremental_update_));
          LOG(INFO) << "mapserver init success!" ;
          if(publish_gridmap_ == true && loadmap_path_ != "") {
              mapserver->load(loadmap_path_);
          }
          pcl_sub   = m_nh->subscribe(pc_topic_,1,&Local_Octomap_Server::PCCallback,this);
          map_pub   = m_nh->advertise<nav_msgs::OccupancyGrid>("projected_map",5,true);
          pcl_pub   = m_nh->advertise<sensor_msgs::PointCloud2>("pointcloud",1,true);
          clear_octomap_srv = m_nh->advertiseService("ClearOctomap", &Local_Octomap_Server::ClearOctomapSrv, this);
          save_gridmap_srv = m_nh->advertiseService("SaveGridmap", &Local_Octomap_Server::SaveGridmapSrv, this);
        }

        bool ClearOctomapSrv(local_octomap_server::ClearOctomap::Request & req, local_octomap_server::ClearOctomap::Response &res) {
            if(req.cmd==true) {
                if(mtx.try_lock()){
                    mapserver->reset();
                    LOG(INFO) << "----------------clear octomap tree!----------------";
                    mtx.unlock();
                }
            }
            return true;
        }

        bool SaveGridmapSrv(local_octomap_server::SaveGridmap::Request & req, local_octomap_server::SaveGridmap::Response &res) {
            if(req.map_savepath!="") {
                res.state = mapserver->save(req.map_savepath);
                return res.state;
            }
            else 
                return false;
        }

        void PCCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg) 
        {
            if(mtx.try_lock()){
                try {
                    geometry_msgs::TransformStamped tf = tf_buf.lookupTransform(base_frame_,pc_msg->header.frame_id,pc_msg->header.stamp,ros::Duration(1.0));
                    Eigen::Matrix4f transform;
                    Tf2Eigen(tf.transform,transform);
                    PointCloudPtr pc = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
                    PointCloudPtr local_pc = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PCLPointCloud2 pcl_pc;
                    pcl_conversions::toPCL(*pc_msg, pcl_pc);
                    pcl::fromPCLPointCloud2(pcl_pc, *local_pc);
                    pcl::transformPointCloud(*local_pc,*pc,transform);

                    Eigen::Vector3d cam(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z);
                    mapserver->insertPCLPointCloud(pc,cam,max_range_);
                    mapserver->getPCLPointCloud(pc);
                    pcl::transformPointCloud(*pc,*local_pc,transform.inverse());
                    sensor_msgs::PointCloud2 msg;
                    pcl::toROSMsg(*local_pc,msg);
                    msg.header = pc_msg->header;
                    pcl_pub.publish(msg);
                    if(publish_gridmap_ == true) {
                        mapserver->processGridMap();
                        mapserver->m_gridmap.header.stamp = pc_msg->header.stamp;
                        mapserver->m_gridmap.header.frame_id = base_frame_;
                        map_pub.publish(mapserver->m_gridmap);
                    }
                } catch (tf2::TransformException &ex) {
                    ROS_WARN_STREAM(ex.what());
                }
                mtx.unlock();
            }
        }
    private:
        ros::NodeHandle *m_nh;
        ros::Subscriber pcl_sub;
        ros::Publisher pcl_pub,map_pub;  
        std::string pc_topic_,base_frame_,loadmap_path_; 
        double leaf_size_,max_range_,max_rayRange_;  
        bool publish_gridmap_,incremental_update_;
        MapServer::Ptr mapserver;
        tf2_ros::Buffer & tf_buf;
        ros::ServiceServer clear_octomap_srv,save_gridmap_srv;
        std::mutex mtx;
};

void initGLOG() {
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = "/home/ict/logs/";
}

int main(int argc,char** argv) {
    google::InitGoogleLogging(argv[0]); 
    initGLOG();
    ros::init(argc, argv, "LocalOctomapServer");
    std::shared_ptr<tf2_ros::Buffer> TfBufPtr;
    TfBufPtr = std::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::Duration(10)));
    tf2_ros::TransformListener listener(*TfBufPtr);
    Local_Octomap_Server server(*TfBufPtr);

    ros::Rate r(2);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    // ros::spin();
    return (0);
}
