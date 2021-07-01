#include "local_octomap_server/MapServer.h"

MapServer::MapServer(double leaf_size, double max_range, bool incremental_update)
{
    tree = OcTreePtr(new octomap::OcTree(leaf_size));
    reset();
    m_gridmap.info.resolution = leaf_size;
    m_maxRange = max_range;
    m_treeDepth = tree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;
    m_incrementalupdate = incremental_update;
}

void MapServer::reset() 
{
    tree->clear();
    // clear 2D map:
    m_gridmap.data.clear();
    m_gridmap.info.height = 0.0;
    m_gridmap.info.width = 0.0;
    m_gridmap.info.resolution = 0.0;
    m_gridmap.info.origin.position.x = 0.0;
    m_gridmap.info.origin.position.y = 0.0;
    m_gridmap.info.origin.position.z = 0.0;
    m_gridmap.info.origin.orientation.x = 0.0;
    m_gridmap.info.origin.orientation.y = 0.0;
    m_gridmap.info.origin.orientation.z = 0.0;
    m_gridmap.info.origin.orientation.w = 1.0;
}
bool MapServer::save(const std::string& filepath_)
{
    if(mkdir(filepath_.c_str(),S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH)<0)
    {
        DLOG(INFO) << "file dir exists!";
    }
    std::string octree_path = filepath_ + "/map.bt";
    LOG(INFO) << octree_path ;
    // if(!saveGridMap(filepath_)) {
    //     return false;
    // }
    if(!saveOcTree(octree_path)) {
        return false;
    }
    LOG(INFO) << "map saved all success!" ;
    return true;
}
bool MapServer::load(const std::string& filepath_)
{
    std::string octree_path = filepath_ + "/map.bt";
    std::string gridmap_path = filepath_ + "/map.yaml";
    // if(!loadGridMap(gridmap_path)) {
    //     return false;
    // }
    if(!loadOcTree(octree_path)) {
        return false;
    }
    LOG(INFO) << "map loaded all success!" ;    
    return true;
}
void MapServer::setOcTree(OcTreePtr octree) 
{
    tree = octree;
}
bool MapServer::saveOcTree(const std::string& filename_)
{
    if (!fopen(filename_.c_str(), "w"))
    {
        LOG(ERROR) << "could not open " << filename_ ;
        return false;
    }
    LOG(INFO) << "saving octree to " << filename_ << "..." ;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    tree->writeBinary(filename_);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double track1= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    LOG(INFO) << "time costs: " << track1 * 1000 <<"ms";
    LOG(INFO) << "octree saved success!" ;
    return true;

}
bool MapServer::loadOcTree(const std::string& filename_)
{
    if (std::ifstream(filename_.c_str()).fail()) {
        LOG(ERROR) << "could not open " << filename_ ;
        return false;
    }
    LOG(INFO) << "loading octree from " << filename_ << "..." ;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    tree->readBinary(filename_);
    bool state = m_incrementalupdate;
    m_incrementalupdate = false;
    processGridMap();
    m_incrementalupdate = state;

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double track1= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    LOG(INFO) << "time costs: " << track1 * 1000 <<"ms";
    int cnt = 0;
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs();it != end; ++it){
        if(tree->isNodeOccupied(*it)){
            cnt++;
        } 
    }
    LOG(INFO) << "octree size:" << cnt ;
    LOG(INFO) << "octree loaded success!" ;
    return true;
}
void MapServer::insertPCLPointCloud(PointCloudPtr local_pc, const Eigen::Vector3d cam, double range)
{ 
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    if(range > 0) {
        for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(); it != tree->end_leafs(); it++)
        {
            octomap::point3d point = it.getCoordinate();
            Eigen::Vector3d p(point.x(),point.y(),point.z());
            double dist = (cam-p).norm();
            if(dist > range){
                tree->deleteNode(point,it.getDepth());
            }
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    octomap::point3d sensorOrigin( cam(0), cam(1), cam(2) );
    if (!tree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
        || !tree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
    {
        ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
    }

    // instead of direct scan insertion
    octomap::KeySet free_cells, occupied_cells;

    for (auto p:local_pc->points) {
        octomap::point3d point(p.x, p.y, p.z);
        // maxrange check
        if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

            // free cells
            if (tree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }
            // occupied endpoint
            octomap::OcTreeKey key;
            if (tree->coordToKeyChecked(point, key)){
                occupied_cells.insert(key);
                updateMinKey(key, m_updateBBXMin);
                updateMaxKey(key, m_updateBBXMax);
            }
        } else {// ray longer than maxrange:;
            octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            if (tree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                octomap::OcTreeKey endKey;
                if (tree->coordToKeyChecked(new_end, endKey)){
                    free_cells.insert(endKey);
                    updateMinKey(endKey, m_updateBBXMin);
                    updateMaxKey(endKey, m_updateBBXMax);
                } else{
                    ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
                }
            }
        }
    }

    // mark free cells only if not seen occupied in this cloud
    for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
        if (occupied_cells.find(*it) == occupied_cells.end()){
            tree->updateNode(*it, false);
        }
    }

    // now mark all occupied cells:
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
        tree->updateNode(*it, true);
    }

    octomap::point3d minPt, maxPt;

    minPt = tree->keyToCoord(m_updateBBXMin);
    maxPt = tree->keyToCoord(m_updateBBXMax);
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    double track1= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    double track2= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();

    DLOG(INFO) << "insert localPointCloud success!"<< local_pc->points.size()<< " points in local_pc";
    LOG(INFO)  << "delete localPointCloud costs: " <<track1*1000 <<"ms";
    LOG(INFO)  << "insert localPointCloud costs: " <<track2*1000 <<"ms";
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
  DLOG(INFO)<<"octree size:"<<pc->points.size();
}

void MapServer::update2DMap(const octomap::OcTree::iterator& it, bool occupied){

    // update 2D map (occupied always overrides):

    if (it.getDepth() == m_maxTreeDepth){
        unsigned idx = mapIdx(it.getKey());
        if (occupied)
        m_gridmap.data[mapIdx(it.getKey())] = 100;
        else if (m_gridmap.data[idx] == -1){
        m_gridmap.data[idx] = 0;
        }

    } else{
        int intSize = 1 << (m_maxTreeDepth - it.getDepth());
        octomap::OcTreeKey minKey=it.getIndexKey();
        for(int dx=0; dx < intSize; dx++){
        int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
        for(int dy=0; dy < intSize; dy++){
            unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/m_multires2DScale);
            if (occupied)
            m_gridmap.data[idx] = 100;
            else if (m_gridmap.data[idx] == -1){
            m_gridmap.data[idx] = 0;
            }
        }
        }
    }
}

void MapServer::adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const{
    if (map.info.resolution != oldMapInfo.resolution){
        ROS_ERROR("Resolution of map changed, cannot be adjusted");
        return;
    }

    int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x)/map.info.resolution +0.5);
    int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y)/map.info.resolution +0.5);
    
    if (i_off < 0 || j_off < 0
        || oldMapInfo.width  + i_off > map.info.width
        || oldMapInfo.height + j_off > map.info.height)
    {
        ROS_ERROR("%d %d %d %d",i_off,j_off,oldMapInfo.width,map.info.width);
        ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
        return;
    }

    nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

    map.data.clear();
    // init to unknown:
    map.data.resize(map.info.width * map.info.height, -1);

    nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

    for (int j =0; j < int(oldMapInfo.height); ++j ){
        // copy chunks, row by row:
        fromStart = oldMapData.begin() + j*oldMapInfo.width;
        fromEnd = fromStart + oldMapInfo.width;
        toStart = map.data.begin() + ((j+j_off)*m_gridmap.info.width + i_off);
        copy(fromStart, fromEnd, toStart);
    }
}

void MapServer::prepareGridMap(){
    // init projected 2D map:
    nav_msgs::MapMetaData oldMapInfo = m_gridmap.info;

    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    tree->getMetricMin(minX, minY, minZ);
    tree->getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = tree->coordToKey(minPt, m_maxTreeDepth);
    octomap::OcTreeKey maxKey = tree->coordToKey(maxPt, m_maxTreeDepth);

    ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

    // add padding if requested (= new min/maxPts in x&y):
    double halfPaddedX = 0.5*m_minSizeX;
    double halfPaddedY = 0.5*m_minSizeY;
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    minPt = octomap::point3d(minX, minY, minZ);
    maxPt = octomap::point3d(maxX, maxY, maxZ);

    octomap::OcTreeKey paddedMaxKey;
    if (!tree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)){
        ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }
    if (!tree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)){
        ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }

    ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);

    m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
    m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;
    int mapOriginX = minKey[0] - m_paddedMinKey[0];
    int mapOriginY = minKey[1] - m_paddedMinKey[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = tree->keyToCoord(m_paddedMinKey, m_treeDepth);
    double gridRes = tree->getNodeSize(m_maxTreeDepth);
    m_projectCompleteMap = (!m_incrementalupdate || (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
    m_gridmap.info.resolution = gridRes;
    m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
    m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
    if (m_maxTreeDepth != m_treeDepth){
        m_gridmap.info.origin.position.x -= m_gridmap.info.resolution/2.0;
        m_gridmap.info.origin.position.y -= m_gridmap.info.resolution/2.0;
    }

    // // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (m_maxTreeDepth < m_treeDepth)
        m_projectCompleteMap = true;


    if(m_projectCompleteMap){
        ROS_DEBUG("Rebuilding complete 2D map");
        m_gridmap.data.clear();
        // init to unknown:
        m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

    } else {

        if (mapChanged(oldMapInfo, m_gridmap.info)){
            ROS_DEBUG("2D grid map size changed to %dx%d", m_gridmap.info.width, m_gridmap.info.height);
            adjustMapData(m_gridmap, oldMapInfo);
        }
        nav_msgs::OccupancyGrid::_data_type::iterator startIt;
        size_t mapUpdateBBXMinX = std::max(size_t(0), (size_t(m_updateBBXMin[0]) - size_t(m_paddedMinKey[0]))/size_t(m_multires2DScale));
        size_t mapUpdateBBXMinY = std::max(size_t(0), (size_t(m_updateBBXMin[1]) - size_t(m_paddedMinKey[1]))/size_t(m_multires2DScale));
        size_t mapUpdateBBXMaxX = std::min(size_t(m_gridmap.info.width-1), (size_t(m_updateBBXMax[0]) - size_t(m_paddedMinKey[0]))/size_t(m_multires2DScale));
        size_t mapUpdateBBXMaxY = std::min(size_t(m_gridmap.info.height-1), (size_t(m_updateBBXMax[1]) - size_t(m_paddedMinKey[1]))/size_t(m_multires2DScale));

        assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
        assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

        size_t numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;
        // test for max idx:
        uint max_idx = m_gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
        if (max_idx  >= m_gridmap.data.size())
            ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, m_gridmap.data.size(), m_gridmap.info.width, m_gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);

        // reset proj. 2D map in bounding box:
        for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j){
            std::fill_n(m_gridmap.data.begin() + m_gridmap.info.width*j+mapUpdateBBXMinX,
                        numCols, -1);
        }
    }
}

void MapServer::processGridMap(){
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    prepareGridMap();
    // now, traverse all leafs in the tree:
    for (octomap::OcTree::iterator it = tree->begin(m_maxTreeDepth); it != tree->end(); ++it)
    {
        bool inUpdateBBX = isInUpdateBBX(it);
        double z = it.getZ();
        double half_size = it.getSize() / 2.0;        
        if (tree->isNodeOccupied(*it)){
            if (m_projectCompleteMap || inUpdateBBX)
            {
                double size = it.getSize();
                double x = it.getX();
                double y = it.getY();
                // Ignore speckles in the map:
                if ((it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
                    ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
                    continue;
                } // else: current octree node is no speckle, send it out
                if(z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ){
                    update2DMap(it, true);
                }
                else if(z - half_size > -m_occupancyMinZ && z + half_size <= m_occupancyMinZ){
                    update2DMap(it, false);
                }
            }
        } else{ // node not occupied => mark as free in 2D map if unknown so far
            if (z - half_size > -m_occupancyMinZ && z - half_size < m_occupancyMaxZ)
            {
                if(m_projectCompleteMap || inUpdateBBX){
                    update2DMap(it, false);
                }
            }
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double track = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    LOG(INFO)  << "insert processGridMap costs: " <<track*1000 <<"ms";
}

bool MapServer::saveGridMap(const std::string & filepath_)
{
    int threshold_occupied = 65;
    int threshold_free = 25;

    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            m_gridmap.info.width,
            m_gridmap.info.height,
            m_gridmap.info.resolution);


    std::string mapdatafile = filepath_ + "/map.pgm";
    ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return false;
    }
    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
            m_gridmap.info.resolution, m_gridmap.info.width, m_gridmap.info.height);
    for(unsigned int y = 0; y < m_gridmap.info.height; y++) {
        for(unsigned int x = 0; x < m_gridmap.info.width; x++) {
            unsigned int i = x + (m_gridmap.info.height - y - 1) * m_gridmap.info.width;
            if (m_gridmap.data[i] >= 0 && m_gridmap.data[i] <= threshold_free) { // [0,free)
                fputc(254, out);
            } else if (m_gridmap.data[i] >= threshold_occupied) { // (occ,255]
                fputc(000, out);
            } else { //occ [0.25,0.65]
                fputc(205, out);
            }
        }
    }
    fclose(out);

    std::string mapmetadatafile = filepath_ + "/map.yaml";
    ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");
    /*
        resolution: 0.100000
        origin: [0.000000, 0.000000, 0.000000]
        #
        negate: 0
        occupied_thresh: 0.65
        free_thresh: 0.196
    */
    geometry_msgs::Quaternion orientation = m_gridmap.info.origin.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);
    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapdatafile.c_str(), m_gridmap.info.resolution, m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y, yaw);
    fclose(yaml);

    ROS_INFO("Done\n");
    return true;
}

bool MapServer::loadGridMap(const std::string& filename_)
{
    std::string mapfname = "";
    double origin[3];
    int negate;
    double res, occ_th, free_th;
    MapMode mode = TRINARY;

    std::ifstream fin(filename_.c_str());
    if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", filename_.c_str());
        return false;
    }

    YAML::Node doc = YAML::Load(fin);

    try {
        res = doc["resolution"].as<double>();
        if (abs(res - m_gridmap.info.resolution)>0.0000001) {
            ROS_ERROR("The resolution of loaded map does not fit the current gridmap.");
            return false;
        } 
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        return false;
    }
    try {
        negate = doc["negate"].as<int>();
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        return false;
    }
    try {
        occ_th = doc["occupied_thresh"].as<double>();
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        return false;
    }
    try {
        free_th = doc["free_thresh"].as<double>();
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        return false;
    }
    try {
        std::string modeS = "";
        modeS = doc["mode"].as<std::string>();
        if(modeS=="trinary")
            mode = TRINARY;
        else if(modeS=="scale")
            mode = SCALE;
        else if(modeS=="raw")
            mode = RAW;
        else{
        ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
        return false;
        }
    } catch (YAML::Exception &) {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        mode = TRINARY;
    }
    try {
        origin[0] = doc["origin"][0].as<double>();
        origin[1] = doc["origin"][1].as<double>();
        origin[2] = doc["origin"][2].as<double>();
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        return false;
    }
    try {
        mapfname = doc["image"].as<std::string>();       
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
            ROS_ERROR("The image tag cannot be an empty string.");
            return false;
        }

        boost::filesystem::path mapfpath(mapfname);
        if (!mapfpath.is_absolute())
        {
            boost::filesystem::path dir(filename_);
            dir = dir.parent_path();
            mapfpath = dir / mapfpath;
            mapfname = mapfpath.string();
        }
    } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        return false;
    }

    ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
    nav_msgs::GetMap::Response map_resp_;
    try
    {
        map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR("%s", e.what());
        exit(-1);
    }
    // To make sure get a consistent time in simulation
    ROS_DEBUG("Waiting for valid time (make sure use_sime_time is false or a clock server (e.g., gazebo) is running)");
    ros::Time::waitForValid();
    map_resp_.map.info.map_load_time = ros::Time::now();
    map_resp_.map.header.frame_id = "map";
    map_resp_.map.header.stamp = ros::Time::now();
    ROS_DEBUG("Got time");
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
            map_resp_.map.info.width,
            map_resp_.map.info.height,
            map_resp_.map.info.resolution);
    m_gridmap = map_resp_.map;
    return true;
}
