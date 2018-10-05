#include <rayBasedSSL.h>

using namespace octomap;
using octomap_msgs::Octomap;

/// Used in the priority queue to sort potential sources
bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace raybased_soundlocalization{    

    /// Contructor
    RaybasedSoundlocalization::RaybasedSoundlocalization(ros::NodeHandle private_nh_)
    : m_nh(),
      PI(3.14159),
      m_worldFrameId("/map"),
      m_latchedTopics(true),
      m_useColoredMap(true),
      m_octree(NULL),
      m_treeDepth(0), m_res(0.05), m_maxTreeDepth(0),
      ps_drawSoundPotentialField(true), // Do you want to publish sound potential field
      ps_queueIndex(0),     
      ps_minEnergy(0.8),       // Thresh hold energy of ray : 0.75(probability), 0.8, 0.9
      ps_ceilingHeight(2.5),    // ceiling height : 2.5m
      ps_temperature(20.0),     // default temperature : 20 degree
      ps_humidity(50.0),        // default humidity : 50%
      ps_minLatitude(-60.0),
      ps_maxLatitude(70.0),
      ps_numberOfParticle(200),
      ps_soundCubeToBaseFootprint(-0.008, -0.002, 0.702)      
    {
        /// You have to modify this project path for your computer environment.
        projectPath = "/home/inkyu/catkin_raybased_ssl_ws/src/reflection_aware_ssl/reflection_aware_ssl/";

        ros::NodeHandle private_nh(private_nh_);
        private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);

        ps_numberOfQueue[0] = 0; ps_numberOfQueue[1] = 0; ps_numberOfQueue[2] = 0; ps_numberOfQueue[3] = 0;

        double r, g, b, a;
        private_nh.param("color/r", r, 0.0);
        private_nh.param("color/g", g, 0.0);
        private_nh.param("color/b", b, 1.0);
        private_nh.param("color/a", a, 1.0);
        m_color.r = r;
        m_color.g = g;
        m_color.b = b;
        m_color.a = a;

        private_nh.param("color_free/r", r, 0.0);
        private_nh.param("color_free/g", g, 1.0);
        private_nh.param("color_free/b", b, 0.0);
        private_nh.param("color_free/a", a, 1.0);
        m_colorFree.r = r;
        m_colorFree.g = g;
        m_colorFree.b = b;
        m_colorFree.a = a;

        private_nh.param("latch", m_latchedTopics, m_latchedTopics);
        if (m_latchedTopics){
            ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
        } else
            ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");

        private_nh.param("temperature", ps_temperature, ps_temperature);
        private_nh.param("humidity", ps_humidity, ps_humidity);


        atmosphericAbsorption.init(ps_temperature, ps_humidity);

        groundTruthPosition[0] = 0.0f;
        groundTruthPosition[1] = 0.0f;
        groundTruthPosition[2] = 0.0f;

        /// Sound Potential Field, which is grid map.
        /// and it contains a traveled acoustic rays.
        ps_soundPotentialField.initPotentialField(ps_ceilingHeight, 0.0, 3);

        /// For visualization
        std::cout << " .. for visualizing" << std::endl;
        m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("rassl_occupied_cells_vis_array", 1, m_latchedTopics);
        m_rayMarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("rassl_occupied_ray_vis_array", 1, m_latchedTopics);
        ps_particlePub = m_nh.advertise<sensor_msgs::PointCloud2>("rassl_particle_points", 1);
        robotPosPublisher = m_nh.advertise<visualization_msgs::Marker>("ra_ssl_robot_pos", 1, m_latchedTopics);
        groundTruthPosPublisher = m_nh.advertise<visualization_msgs::Marker>("ra_ssl_groundtruth_pos", 1, m_latchedTopics);
        /// Visualize estimated source position
        ps_visualParticleToModule = m_nh.advertise<reflection_aware_ssl_message::visualizeConvergenceParticle>("rassl_to_visualDetector", 1, m_latchedTopics);

        /// Update Octomap from .bt file        
        m_octree = new OcTreeT(m_res);
        if(m_octree->readBinary(projectPath + "data/octomap_stationary_source.bt") == -1){
            ROS_INFO("Couldn't read octomap binary file.");
        }        
        m_res = m_octree->getResolution();
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;

        m_octree->getMetricMax(max_x, max_y, max_z);
        m_octree->getMetricMin(min_x, min_y, min_z);

        /// Potential sources from TDOA-based method
        std::cout << " .. for initializing subscribers" << std::endl;
        ps_potentialSourcesSub = m_nh.subscribe("/dir_info", 100, &RaybasedSoundlocalization::insertPotentialsourcesCallback, this);

        /// Subscribe the source position (ground truth)
        ps_sourcePositionSubscriber = m_nh.subscribe("/source_ground_truth", 100, &RaybasedSoundlocalization::insertSourcePositionCallback, this);

        /// Intialize timer for Particle filter
        particleFilter_timer = m_nh.createTimer(ros::Duration(0.05), &RaybasedSoundlocalization::particleFilterCallback, this);

        /// Intialize timer for traveling the aocustic rays
        /// Propagate acoustic rays per 0.5sec.
        extractPS_timer = m_nh.createTimer(ros::Duration(0.5), &RaybasedSoundlocalization::extractPotentialSourceCallback, this);


        octomap::point3d maxMap = octomap::point3d(max_x, max_y, ps_ceilingHeight);
        octomap::point3d minMap = octomap::point3d(min_x, min_y, 0.0);

        /// Initalize Particle filter
        ps_particleFilter = new ParticleFilter(ps_numberOfParticle, 50, 3.0, 0.7, maxMap, minMap, ps_minEnergy);
        ps_particleFilter->setOctreeInf(m_octree);

        ROS_INFO(">> End initializing");

    }

    /// Destructor
    RaybasedSoundlocalization::~RaybasedSoundlocalization(){
        if (m_octree) {
            delete m_octree;         
        }     

        ROS_INFO("Destroyed RaybasedSoundLocalization");
    }

    /// Receive the ground truth position of the source.
    void RaybasedSoundlocalization::insertSourcePositionCallback(const tdoa_ra_ssl_message::sourceGroundTruthConstPtr& sourcePosPtr)
    {
        float tmpPosX = sourcePosPtr->source_pos.x;
        float tmpPosY = sourcePosPtr->source_pos.y;
        float tmpPosZ = sourcePosPtr->source_pos.z;

        groundTruthPosition[0] = tmpPosX;
        groundTruthPosition[1] = tmpPosY;
        groundTruthPosition[2] = tmpPosZ;

        /// Publish the ground truth and robot position to Rviz for visualization.
        publishCylinders(octomap::point3d(tmpPosX, tmpPosY, tmpPosZ), octomap::point3d(0.0f, 0.0f, 0.0f));
    }


    /// Callback function when potential sources, directions of sound, are inserted from TDOA-based method
    void RaybasedSoundlocalization::insertPotentialsourcesCallback(const tdoa_ra_ssl_message::PotentialSourcesConstPtr& potentialSources_ptr)
    {
        /// Subscribe the potential sources, which is the directions of the sound.

        /// size of potential sources
        int sizeOfPotentialSources = potentialSources_ptr->potential_sources.size();

        /// insert PotentialSourcesPtr to Queue
        tdoa_ra_ssl_message::PotentialSourcesAverage tempPotentialSources;

        tempPotentialSources.header.stamp = potentialSources_ptr->header.stamp;
        tempPotentialSources.header.frame_id = potentialSources_ptr->header.frame_id;
        tempPotentialSources.frequency = potentialSources_ptr->frequency;

        float averageProbability = 0;

        for(int i=0 ; i<sizeOfPotentialSources ; i++)
        {            
            tdoa_ra_ssl_message::SourceInfoT potentialSourceNode;

            potentialSourceNode.source_id = potentialSources_ptr->potential_sources[i].source_id;
            potentialSourceNode.source_pos.x = potentialSources_ptr->potential_sources[i].source_pos.x;
            potentialSourceNode.source_pos.y = potentialSources_ptr->potential_sources[i].source_pos.y;
            potentialSourceNode.source_pos.z = potentialSources_ptr->potential_sources[i].source_pos.z;
            potentialSourceNode.longitude = potentialSources_ptr->potential_sources[i].longitude;
            potentialSourceNode.latitude = potentialSources_ptr->potential_sources[i].latitude;
            potentialSourceNode.source_probability = potentialSources_ptr->potential_sources[i].source_probability;
            potentialSourceNode.source_energy = potentialSources_ptr->potential_sources[i].source_energy;

            potentialSourceNode.robot_pos.x = 0.0f;
            potentialSourceNode.robot_pos.y = 0.0f;
            potentialSourceNode.robot_pos.z = 0.0f;
            potentialSourceNode.robot_rotate.x = 0.0f;
            potentialSourceNode.robot_rotate.y = 0.0f;
            potentialSourceNode.robot_rotate.z = 0.0f;
            potentialSourceNode.groundTruth_pos.x = groundTruthPosition[0];
            potentialSourceNode.groundTruth_pos.y = groundTruthPosition[1];
            potentialSourceNode.groundTruth_pos.z = groundTruthPosition[2];
            averageProbability += potentialSources_ptr->potential_sources[i].source_probability;    // sum of probabilities
            tempPotentialSources.potential_sources.push_back(potentialSourceNode);
        }

        tempPotentialSources.average_probability = averageProbability;

        /// Save the potential source (direction info.) to the queue to handle it every 0.5sec.
        if(ps_potentialSourcesQueue.size() < 500)
        {            
            ps_numberOfQueue[ps_queueIndex]++;
            ps_potentialSourcesQueue.push_back(tempPotentialSources);
        }
    }

    /// Extract Potential Sources from a traditional sound source localization every 0.5sec
    void RaybasedSoundlocalization::extractPotentialSourceCallback(const ros::TimerEvent& event)
    {
        /* Calculate a computation time */
        struct timeval start_point, end_point;
        gettimeofday(&start_point, NULL);
        /* Starting point */

        /// publish color Marker array to visualize
        publishColorMarker();

        ps_visualParticleToModule.publish(positionToVisualDetector);

        /// Check the error distance between the ground truth and the estimated position.
        octomap::point3d tmpGroundTruthPos(groundTruthPosition[0], groundTruthPosition[1], groundTruthPosition[2]);
        octomap::point3d tmpEstimatedPos(positionToVisualDetector.p_x, positionToVisualDetector.p_y, positionToVisualDetector.p_z);
        if(positionToVisualDetector.s_x != -1){
            float errorDistance = tmpEstimatedPos.distance(tmpGroundTruthPos);
            std::cout << "  < Error distance: " << errorDistance << " >" << std::endl;
        }

        //std::cout << " !Extract Potential Source Callback" << std::endl;

        unsigned nextQueueIndex = ps_queueIndex + 1;
        if(nextQueueIndex == 2)
            nextQueueIndex = 0;

        /// waiting for accumulating potential sources
        if(ps_numberOfQueue[0]==0 || ps_numberOfQueue[1] == 0) {
            std::cout << "waiting for getting potential sources, next queue index: " << nextQueueIndex << std::endl;
            ps_queueIndex = nextQueueIndex;
            return;
        }

        /// Sorting the queue of potential sources according to the average energy of themselves.
        auto comp = [] (PotentialSource &a, PotentialSource &b) -> bool { return a.potentialSourceNode.source_probability  < b.potentialSourceNode.source_probability; };
        std::priority_queue<PotentialSource,std::vector<PotentialSource>, decltype(comp) > highIntensityDirection (comp);

        std::vector<PotentialSource> topPotentialSources;    

        for(std::vector<tdoa_ra_ssl_message::PotentialSourcesAverage>::iterator it = ps_potentialSourcesQueue.begin()
            ; it != ps_potentialSourcesQueue.end() ; it++) {

            for(int i=0 ; i<it->potential_sources.size() ; i++){
                PotentialSource tempPotentialSource(it->potential_sources[i], it->frequency);
                highIntensityDirection.push(tempPotentialSource);
            }
        }

        std::cout << "highIntensityDirection: " << highIntensityDirection.size() << std::endl;
        topPotentialSources.resize(highIntensityDirection.size());

        ps_soundPotentialField.resetPotentialField(m_octree);

        if(highIntensityDirection.size() == 0){
        }else
            {

            int rayIndex = 0;

            /// from (maxidx-window size) to (maxidx), default window size : 60
            for(int i=0 ; i<topPotentialSources.size() ; i++){

                PotentialSource tempPotentialSource = highIntensityDirection.top();
                highIntensityDirection.pop();
                float f = tempPotentialSource.frequency;

                topPotentialSources[i] = tempPotentialSource;

                /// Exception condition, when the directions came from our robot.
                if(tempPotentialSource.potentialSourceNode.latitude < ps_minLatitude
                        || tempPotentialSource.potentialSourceNode.latitude > ps_maxLatitude
                        || i > 10)
                    continue;
                if(tempPotentialSource.potentialSourceNode.source_probability > ps_minEnergy) {

                    point3d origin(tempPotentialSource.potentialSourceNode.robot_pos.x + ps_soundCubeToBaseFootprint.x(),
                                   tempPotentialSource.potentialSourceNode.robot_pos.y + ps_soundCubeToBaseFootprint.y(),
                                   tempPotentialSource.potentialSourceNode.robot_pos.z + ps_soundCubeToBaseFootprint.z());
                    point3d direction(tempPotentialSource.potentialSourceNode.source_pos.x,
                                      tempPotentialSource.potentialSourceNode.source_pos.y,
                                      tempPotentialSource.potentialSourceNode.source_pos.z);

                    double roll = tempPotentialSource.potentialSourceNode.robot_rotate.x;
                    double pitch = tempPotentialSource.potentialSourceNode.robot_rotate.y;
                    double yaw = tempPotentialSource.potentialSourceNode.robot_rotate.z;
                    direction.rotate_IP(roll, pitch, yaw);                    

                    // Get key value of ceiling & bottom
                    // Set initial energy of potential source
                    float initEnergy = tempPotentialSource.potentialSourceNode.source_energy;
                    float initProb = tempPotentialSource.potentialSourceNode.source_probability;

                    // Travels the rays on Potential Field.
                    ps_soundPotentialField.traverseSoundRay(initProb, origin, direction, 7.0, m_octree, rayIndex, atmosphericAbsorption, f);

                    rayIndex++;

                }

            }
            ps_soundPotentialField.setNumberOfRays(rayIndex);
        }

        /// Control the queue of potential
        //ps_potentialSourcesQueue.clear();
        ps_potentialSourcesQueue.erase(ps_potentialSourcesQueue.begin(), ps_potentialSourcesQueue.begin() + ps_numberOfQueue[nextQueueIndex]);
        ps_numberOfQueue[nextQueueIndex] = 0;
        ps_queueIndex = nextQueueIndex;        


        /* Calculate a computation time */
        gettimeofday(&end_point, NULL);
        double operation_time = (double)(end_point.tv_sec)+(double)(end_point.tv_usec)/1000000.0-(double)(start_point.tv_sec)-(double)(start_point.tv_usec)/1000000.0;
        std::cout << "extractPotentialSourceCallback operation time: " << operation_time << std::endl;
        /* End point */
    }

    /// Find a convergin region of acoustic rays using Particle filter
    void RaybasedSoundlocalization::particleFilterCallback(const ros::TimerEvent& event)
    {
        /* Calculate a computation time */
        struct timeval start_point, end_point;
        gettimeofday(&start_point, NULL);
        /* Starting point */

        /// Sampling
        ps_particleFilter->predictNextPositionOfParticle();

        /// Compute the particle's weights
        ps_particleFilter->computeImportanceSamplingWeight(ps_soundPotentialField.PotentialRays);

        /// Publish particles for visualization
        PointCloud::Ptr msg (new PointCloud);
        sensor_msgs::PointCloud2 cloud2;
        msg->header.frame_id = m_worldFrameId;
        msg->height = 1;
        msg->width = ps_numberOfParticle;
        ps_particleFilter->transferParticleToPointCloud(msg->points);
        pcl::toROSMsg(*msg, cloud2);        
        ps_particlePub.publish(cloud2);      

        /// resample particles based on weigts.
        if(ps_soundPotentialField.PotentialRays.size() <= 1){
            //std::cout << "\t PotentialSources are empty" << std::endl;
        }
        else{
            ps_particleFilter->resampleParticles();
        }

        /// Compute Generalized Value to determine whether particles are converged or not.
        octomap::point3d targetPos;
        double determinantCov;
        tf::Matrix3x3 covM;        
        bool ps_positionDected = ps_particleFilter->getSoundSourcePosition(targetPos, determinantCov, covM);

        /// If particles are converged, which means there is a sound source.
        if(!ps_positionDected){ /// When there is no sound sourse.
            positionToVisualDetector.p_x = 0;
            positionToVisualDetector.p_y = 0;
            positionToVisualDetector.p_z = 0;
            positionToVisualDetector.q_x = 0;
            positionToVisualDetector.q_y = 0;
            positionToVisualDetector.q_z = 0;
            positionToVisualDetector.q_w = 0;
            positionToVisualDetector.s_x = -1;
            positionToVisualDetector.s_y = -1;
            positionToVisualDetector.s_z = -1;
            return;
        }


        /// There is a sound source.
        /// We compute the probability distribution of sound source position
        /// Get Rotation of sphere
        Eigen::Matrix3f tempM(3,3);
        tempM << covM.getRow(0).x(), covM.getRow(0).y(), covM.getRow(0).z(),
                covM.getRow(1).x(), covM.getRow(1).y(), covM.getRow(1).z(),
                covM.getRow(2).x(), covM.getRow(2).y(), covM.getRow(2).z();        

        Eigen::EigenSolver<Eigen::Matrix3f> tempM_Solver;
        tempM_Solver.compute(tempM);

        octomap::point3d eigx_v = octomap::point3d(tempM_Solver.eigenvectors().row(0)[0].real(), tempM_Solver.eigenvectors().row(0)[1].real(), tempM_Solver.eigenvectors().row(0)[2].real());
        octomap::point3d eigy_v = octomap::point3d(tempM_Solver.eigenvectors().row(1)[0].real(), tempM_Solver.eigenvectors().row(1)[1].real(), tempM_Solver.eigenvectors().row(1)[2].real());
        octomap::point3d eigz_v = octomap::point3d(tempM_Solver.eigenvectors().row(2)[0].real(), tempM_Solver.eigenvectors().row(2)[1].real(), tempM_Solver.eigenvectors().row(2)[2].real());

        double eigx_n = eigx_v.norm();  double eigy_n = eigy_v.norm();  double eigz_n = eigz_v.norm();

        octomap::point3d eigValue = octomap::point3d(tempM_Solver.eigenvalues().data()[0].real(), tempM_Solver.eigenvalues().data()[1].real(), tempM_Solver.eigenvalues().data()[2].real());
        tf::Quaternion q(eigz_n, eigy_n, eigx_n);

        /// "positionToVisualDetector" contains information of the probability distribution of sound source position
        /// "q_x, y, z" are the mean position
        /// "s_x, y, z" are the variance
        positionToVisualDetector.p_x = targetPos.x();
        positionToVisualDetector.p_y = targetPos.y();
        positionToVisualDetector.p_z = targetPos.z();
        positionToVisualDetector.q_x = q.x();
        positionToVisualDetector.q_y = q.y();
        positionToVisualDetector.q_z = q.z();
        positionToVisualDetector.q_w = q.w();
        positionToVisualDetector.s_x = eigValue.x();
        positionToVisualDetector.s_y = eigValue.y();
        positionToVisualDetector.s_z = eigValue.z();

        /* Calculate a computation time */
        gettimeofday(&end_point, NULL);
        double operation_time = (double)(end_point.tv_sec)+(double)(end_point.tv_usec)/1000000.0-(double)(start_point.tv_sec)-(double)(start_point.tv_usec)/1000000.0;
        //std::cout << "Particle filter operation time: " << operation_time << std::endl;
        /* End point */
    }



    /// Visualize the acoustic rays and map data (obstacles).
    void RaybasedSoundlocalization::publishColorMarker(const ros::Time &rostime)
    {
        size_t octomapSize = m_octree->size();
        if(octomapSize <= 1)
        {
            ROS_WARN("ColorMarker isn't published, octomap is empty");
            return;
        }

        bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);

        visualization_msgs::MarkerArray occupiedNodesVis;
        occupiedNodesVis.markers.resize(m_treeDepth+1);

        for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it)
        {
            if (m_octree->isNodeOccupied(*it)){
                double size = it.getSize();
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();
#ifdef COLOR_OCTOMAP_SERVER
                int r = it->getColor().r;
                int g = it->getColor().g;
                int b = it->getColor().b;
#endif

                //create marker:
                if(publishMarkerArray)
                {
                    unsigned idx = it.getDepth();
                    assert(idx < occupiedNodesVis.markers.size());

                    geometry_msgs::Point cubeCenter;
                    cubeCenter.x = x;
                    cubeCenter.y = y;
                    cubeCenter.z = z;

                    occupiedNodesVis.markers[idx].points.push_back(cubeCenter);                

#ifdef COLOR_OCTOMAP_SERVER
                    if (m_useColoredMap) {
                        std_msgs::ColorRGBA _color;
                        //_color.r = (r / 255.); _color.g = (g / 255.); _color.b = (b / 255.); _color.a = 1.0;
                        _color.r = 0.4f; _color.g = 0.4f; _color.b = 0.4f; _color.a = 1.0;
                        occupiedNodesVis.markers[idx].colors.push_back(_color);
                    }
#endif
                }
            }
        }


        visualization_msgs::MarkerArray occupiedRayNodesVis;
        occupiedRayNodesVis.markers.resize(m_treeDepth+1);

        if(ps_drawSoundPotentialField)
        {
            int sizeOfPotentialRays = ps_soundPotentialField.PotentialRays.size();
            //std::cout << "sizeOfPotentialRays: " << sizeOfPotentialRays << std::endl;
            for(int i=0 ; i<sizeOfPotentialRays ; i++){
                int sizeOfRay = ps_soundPotentialField.PotentialRays[i].size();
                //std::cout << "  sizeOfRay: " << sizeOfRay << std::endl;
                for(int j=0 ; j<sizeOfRay ; j++){
                    ps_soundPotentialField.PotentialRays[i].ray[j].k[0];
                    octomap::OcTreeKey keyOfRay(ps_soundPotentialField.PotentialRays[i].ray[j].k[0],
                            ps_soundPotentialField.PotentialRays[i].ray[j].k[1],
                            ps_soundPotentialField.PotentialRays[i].ray[j].k[2]);
                    octomap::point3d positionOfRay = m_octree->keyToCoord(keyOfRay, m_maxTreeDepth);

                    int orderOfReflection = ps_soundPotentialField.PotentialRays[i].ray[j].reflectIndex;

                    unsigned idx = m_maxTreeDepth;
                    assert(idx < occupiedNodesVis.markers.size());

                    geometry_msgs::Point cubeCenter;
                    cubeCenter.x = positionOfRay.x();
                    cubeCenter.y = positionOfRay.y();
                    cubeCenter.z = positionOfRay.z();

                    occupiedRayNodesVis.markers[idx].points.push_back(cubeCenter);

#ifdef COLOR_OCTOMAP_SERVER
                    if (m_useColoredMap) {                        
                        std_msgs::ColorRGBA color_;

                        if(orderOfReflection > 0){  /// Reflection ray
                            color_.r = 1.0f;
                            color_.g = 0;
                            color_.b = 0;
                        }else{  /// Direct ray
                            color_.r = 0.0f;
                            color_.g = 0.0f;
                            color_.b = 1.0f;
                        }

                        color_.a = 0.5;
                        occupiedRayNodesVis.markers[idx].colors.push_back(color_);
                    }
#endif

                }
            }

        }
        if(publishMarkerArray)
        {
            for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
                occupiedNodesVis.markers[i].header.stamp = rostime;
                occupiedNodesVis.markers[i].ns = "map";
                occupiedNodesVis.markers[i].id = i;
                occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                occupiedNodesVis.markers[i].scale.x = size;
                occupiedNodesVis.markers[i].scale.y = size;
                occupiedNodesVis.markers[i].scale.z = size;
                if (!m_useColoredMap)
                    occupiedNodesVis.markers[i].color = m_color;


                if (occupiedNodesVis.markers[i].points.size() > 0)
                    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
                else
                    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
            }

            m_markerPub.publish(occupiedNodesVis);



            for (unsigned i= 0; i < occupiedRayNodesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                occupiedRayNodesVis.markers[i].header.frame_id = m_worldFrameId;
                occupiedRayNodesVis.markers[i].header.stamp = rostime;
                occupiedRayNodesVis.markers[i].ns = "map";
                occupiedRayNodesVis.markers[i].id = i;
                occupiedRayNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                occupiedRayNodesVis.markers[i].scale.x = size;
                occupiedRayNodesVis.markers[i].scale.y = size;
                occupiedRayNodesVis.markers[i].scale.z = size;
                if (!m_useColoredMap)
                    occupiedRayNodesVis.markers[i].color = m_color;


                if (occupiedRayNodesVis.markers[i].points.size() > 0)
                    occupiedRayNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
                else
                    occupiedRayNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
            }

            m_rayMarkerPub.publish(occupiedRayNodesVis);

        }

    }   // the end of "RaybasedSoundlocalization::publishColorMarker" function


    /// Visualize ground truth and robot position
    void RaybasedSoundlocalization::publishCylinders(octomap::point3d inputGroundTruth, octomap::point3d inputRobotPos)
    {

        /// for ground truth
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = inputGroundTruth.x();
        cubeCenter.y = inputGroundTruth.y();
        cubeCenter.z = inputGroundTruth.z();

        visualization_msgs::Marker groundTruthNodeVis, robotNodeVis;
        ros::Time currTime = ros::Time::now();
        groundTruthNodeVis.header.frame_id = m_worldFrameId;
        groundTruthNodeVis.header.stamp = currTime;
        groundTruthNodeVis.ns = "rassl_ground_truth_pos";
        groundTruthNodeVis.id = 0;
        groundTruthNodeVis.type = visualization_msgs::Marker::CUBE;
        groundTruthNodeVis.scale.x = 0.2;
        groundTruthNodeVis.scale.y = 0.2;
        groundTruthNodeVis.scale.z = 0.2;

        groundTruthNodeVis.pose.position = cubeCenter;

        std_msgs::ColorRGBA _color;
        _color.r = 0.0f; _color.g = 1.0f; _color.b = 0.0f; _color.a = 1.;

        groundTruthNodeVis.color = _color;
        groundTruthNodeVis.action = visualization_msgs::Marker::ADD;
        groundTruthPosPublisher.publish(groundTruthNodeVis);


        /// for robot
        cubeCenter.x = inputRobotPos.x();
        cubeCenter.y = inputRobotPos.y();
        cubeCenter.z = inputRobotPos.z();

        robotNodeVis.header.frame_id = m_worldFrameId;
        robotNodeVis.header.stamp = currTime;
        robotNodeVis.ns = "rassl_robot_pos";
        robotNodeVis.id = 0;
        robotNodeVis.type = visualization_msgs::Marker::CUBE;
        robotNodeVis.scale.x = 0.2;
        robotNodeVis.scale.y = 0.2;
        robotNodeVis.scale.z = 0.2;

        robotNodeVis.pose.position = cubeCenter;

        _color.r = 1.0f; _color.g = 0.0f; _color.b = 1.0f; _color.a = 1.;

        robotNodeVis.color = _color;
        robotNodeVis.action = visualization_msgs::Marker::ADD;
        robotPosPublisher.publish(robotNodeVis);


    }


}   // the end of "raybased_soundlocalization" namespace
