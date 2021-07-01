#pragma once

#include "ros/console.h"
#include "ros/ros.h"
#include "map_server/image_loader.h"
#include "yaml-cpp/yaml.h"
#include <tf/tf.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <nav_msgs/OccupancyGrid.h>
#include <chrono>
#include <dirent.h>
#include <map>
#include <glog/logging.h>

typedef std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudPtr;
typedef std::shared_ptr<octomap::OcTree> OcTreePtr;
class MapServer
{
    public:
        typedef std::shared_ptr<MapServer> Ptr;
        MapServer(double leaf_size=0.02,double max_range=2.0,bool incremental_update=false);
        void reset();
        bool save(const std::string& filepath_);
        bool load(const std::string& filepath_);
        void setOcTree(OcTreePtr octree);
        void insertPCLPointCloud(PointCloudPtr local_pc, const Eigen::Vector3d cam, double range=3.0);
        void getOctomapMsg(octomap_msgs::Octomap& mymap_msg);
        void getPCLPointCloud(PointCloudPtr pc);
        void processGridMap();
        bool saveOcTree(const std::string& filename_);
        bool loadOcTree(const std::string& filename_);
        bool saveGridMap(const std::string& filepath_);
        bool loadGridMap(const std::string& filename_);
        //for 2D gridmap
        nav_msgs::OccupancyGrid m_gridmap;

    protected:
        inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo) {
            return (    oldMapInfo.height != newMapInfo.height
                        || oldMapInfo.width != newMapInfo.width
                        || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                        || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
        }
        inline unsigned mapIdx(int i, int j) const {
            return m_gridmap.info.width * j + i;
        }

        inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
            return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                        (key[1] - m_paddedMinKey[1]) / m_multires2DScale);

        }
        inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
            for (unsigned i = 0; i < 3; ++i)
            min[i] = std::min(in[i], min[i]);
        };

        inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
            for (unsigned i = 0; i < 3; ++i)
            max[i] = std::max(in[i], max[i]);
        };
        /// Test if key is within update area of map (2D, ignores height)
        inline bool isInUpdateBBX(const octomap::OcTree::iterator& it) const {
            // 2^(tree_depth-depth) voxels wide:
            unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
            octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
            return (key[0] + voxelWidth >= m_updateBBXMin[0]
                    && key[1] + voxelWidth >= m_updateBBXMin[1]
                    && key[0] <= m_updateBBXMax[0]
                    && key[1] <= m_updateBBXMax[1]);
        }
        inline bool isSpeckleNode(const octomap::OcTreeKey&nKey) const {
            octomap::OcTreeKey key;
            bool neighborFound = false;
            for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
                for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
                for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
                    if (key != nKey){
                    octomap::OcTreeNode* node = tree->search(key);
                    if (node && tree->isNodeOccupied(node)){
                        // we have a neighbor => break!
                        neighborFound = true;
                    }
                    }
                }
                }
            }
            return neighborFound;
        }

    private:
        void update2DMap(const octomap::OcTree::iterator& it, bool occupied);
        void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo)const;
        void prepareGridMap();
        OcTreePtr tree;
        //for 2D gridmap
        bool m_projectCompleteMap;
        bool m_incrementalupdate;
        unsigned m_treeDepth;
        unsigned m_maxTreeDepth;
        octomap::KeyRay m_keyRay;
        double m_occupancyMinZ = 0.05;
        double m_occupancyMaxZ = 2.0;
        double m_maxRange;
        double m_minSizeX = 1.0;
        double m_minSizeY = 1.0;
        unsigned m_multires2DScale;
        octomap::OcTreeKey m_paddedMinKey;
        octomap::OcTreeKey m_updateBBXMin;
        octomap::OcTreeKey m_updateBBXMax;
};