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
      m_worldFrameId("/map"), m_odomFrameId("/odom"), m_baseFrameId("base_footprint"), ps_SoundCubeFrameId("/micro_center_link"),
      m_groundTruthFrameId("/base_footprint_sound"), m_soundOdomFrameId("/odom_sound"),      
      m_octomapFilename("octomap_"), ps_potentialSourcesFilename("potential_sources_"),
      m_latchedTopics(true),
      m_useColoredMap(true),
      m_octree(NULL),
      m_treeDepth(0),
      m_maxTreeDepth(0),      
      ps_drawFullPotentialRay(false),    // Do you want to publish potential rays which is grid-based
      ps_drawSoundPotentialField(true), // Do you want to publish sound potential field
      ps_queueIndex(0),
      ps_windowSize(60),        // Window size of frame to select rays
      ps_minEnergy(0.8),       // Thresh hold energy of ray : 0.75(probability), 0.8, 0.9
      ps_ceilingHeight(2.5),    // ceiling height : 2.5m
      ps_temperature(20.0),     // default temperature : 20 degree
      ps_humidity(50.0),        // default humidity : 50%
      ps_minLatitude(-60.0),
      ps_maxLatitude(70.0),
      ps_filterWindowSize(0.3),
      ps_numberOfParticle(200),
      ps_InsertPSCounter(0),      
      ps_soundCubeToBaseFootprint(-0.008, -0.002, 0.702)
      //targetPosition(0.0f, 0.0f), latchedRobotControl(false)
    {
        ros::NodeHandle private_nh(private_nh_);
        private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
        private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);

        ps_queueNumber[0] = 0; ps_queueNumber[1] = 0;
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

        /// Sound Potential Field, which is grid map.
        /// and it contains a traveled acoustic rays.
        ps_soundPotentialField.initPotentialField(ps_ceilingHeight, 0.0, 3, ps_filterWindowSize);

        /// For visualization
        m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("raybased_occupied_cells_vis_array", 1, m_latchedTopics);
        m_rayMarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("raybased_occupied_ray_vis_array", 1, m_latchedTopics);
        ps_particlePub = m_nh.advertise<sensor_msgs::PointCloud2>("particles_point", 1);

        ps_visualParticleToModule = m_nh.advertise<reflection_aware_ssl_message::visualizeConvergenceParticle>("raybased_to_visualDetector", 1, m_latchedTopics);

        /// Subscribe data
        /// Octomap from SLAM
        m_octomapfullSub = new message_filters::Subscriber<octomap_msgs::Octomap> (m_nh, "rtabmap/octomap_full", 5);
        m_tfOctomapfullSub = new tf::MessageFilter<octomap_msgs::Octomap> (*m_octomapfullSub, m_tfListener, m_worldFrameId, 5);
        m_tfOctomapfullSub->registerCallback( boost::bind(&RaybasedSoundlocalization::insertOctomapfullCallback, this, _1) );

        /// Potential sources from TDOA-based method
        ps_potentialSourcesSub = m_nh.subscribe("Potential_Sources", 100, &RaybasedSoundlocalization::insertPotentialsourcesCallback, this);                              

        /// Intialize timer for Particle filter
        particleFilter_timer = m_nh.createTimer(ros::Duration(0.05), &RaybasedSoundlocalization::particleFilterCallback, this);

        /// Intialize timer for traveling the aocustic rays
        extractPS_timer = m_nh.createTimer(ros::Duration(0.5), &RaybasedSoundlocalization::extractPotentialSourceCallback, this);


        octomap::point3d maxMap = octomap::point3d(1.5, 1.5, ps_ceilingHeight);
        octomap::point3d minMap = octomap::point3d(-1.5, -1.5, 0.0);

        /// Initalize Particle filter
        ps_particleFilter = new ParticleFilter(ps_numberOfParticle, 50, 3.0, 0.7, maxMap, minMap, ps_minEnergy);


        ps_gaussianParticlePub[0] = m_nh.advertise<visualization_msgs::Marker>("raybased_gaussian_particles1", 1, m_latchedTopics);
        ps_gaussianParticlePub[1] = m_nh.advertise<visualization_msgs::Marker>("raybased_gaussian_particles2", 1, m_latchedTopics);
        ps_gaussianParticlePub[2] = m_nh.advertise<visualization_msgs::Marker>("raybased_gaussian_particles3", 1, m_latchedTopics);
        ps_gaussianParticlePub[3] = m_nh.advertise<visualization_msgs::Marker>("raybased_gaussian_particles4", 1, m_latchedTopics);


        ROS_INFO(">> End initializing");

        check_point.tv_sec = 0;
        check_point.tv_usec = 0;
    }

    /// Destructor
    RaybasedSoundlocalization::~RaybasedSoundlocalization(){
        if (m_octomapfullSub) {
            delete m_octomapfullSub;
        }
        if (m_tfOctomapfullSub) {
            delete m_tfOctomapfullSub;
        }


        if (m_octree) {
            delete m_octree;         
        }     

        ROS_INFO("Destroyed RaybasedSoundLocalization\n");
    }



    /// Callback function when octomap is inserted from SLAM
    void RaybasedSoundlocalization::insertOctomapfullCallback(const octomap_msgs::Octomap::ConstPtr& octomap_full)
    {        
        if(octomap_full == NULL) {
            std::cout << "waiting for getting octomap_full" << std::endl;
            return;
        }
        try {
            m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, octomap_full->header.stamp, ps_baseFootprintToWorldTf);
        } catch(tf::TransformException& ex){
            ROS_ERROR_STREAM( "Transform error of m_baseFrameId: " << ex.what() << ", quitting callback");
        }

        /// get Octomap
        if(m_octree != NULL) {
            m_octree->clear();
            m_octree = NULL;
        }


        octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(* octomap_full);

        m_res = octomap_full->resolution;
        m_octree = dynamic_cast <OcTreeT*> (tree);
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;

        // re-init Particle filter
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;

        m_octree->getMetricMax(max_x, max_y, max_z);
        m_octree->getMetricMin(min_x, min_y, min_z);

        octomap::point3d maxMap = octomap::point3d(max_x, max_y, ps_ceilingHeight);
        octomap::point3d minMap = octomap::point3d(min_x, min_y, 0.0);

        //octomap::point3d maxMap = octomap::point3d(max_x, max_y, max_z);
        //octomap::point3d minMap = octomap::point3d(min_x, min_y, min_z);

        ps_particleFilter->reInitParticleFilter(maxMap, minMap);
        ps_particleFilter->setOctreeInf(m_octree);

        std::cout << "got Octomapfull" << std::endl;

    }


    /// Callback function when potential sources, directions of sound, are inserted from TDOA-based method
    void RaybasedSoundlocalization::insertPotentialsourcesCallback(const tdoa_based_ssl_message::PotentialSourcesConstPtr& potentialSources_ptr)
    {        

        /// Subscribe TF of potential sources, which is robot origin
        /// And subscribe TF of ground truth, a position of a sound source, to extract the accuracy.
        //ros::Time currTime = ros::Time(0);
        ros::Time currTime = potentialSources_ptr->header.stamp;
        tf::StampedTransform robotOriginFullToWorldTf;
        tf::StampedTransform groundTruthToSoundOdom;
        tf::StampedTransform soundOdomToWorldTf;

        ps_audio_get_time = currTime;        
        /// Subscribe TF of potential sources, which is robot origin
        try {
            m_tfOdomListener.lookupTransform(m_worldFrameId, m_baseFrameId, ros::Time(0), robotOriginFullToWorldTf);            
        } catch(tf::TransformException& ex){
            ROS_ERROR_STREAM( "Transform error of sound cube data: " << ex.what() << ", quitting callback");
        }
        /// And subscribe TF of ground truth, a position of a sound source, to extract the accuracy.
        try {            
            m_tfOdomListener.lookupTransform(m_soundOdomFrameId, m_groundTruthFrameId, ros::Time(0), groundTruthToSoundOdom);            
        } catch(tf::TransformException& ex){            
            groundTruthToSoundOdom.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        }
        try {            
            m_tfOdomListener.lookupTransform(m_worldFrameId, m_soundOdomFrameId, ros::Time(0), soundOdomToWorldTf);            
        } catch(tf::TransformException& ex){            
            soundOdomToWorldTf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        }
        /// ps_groundTruth is the real sound source position.
        ps_groundTruth.x() = groundTruthToSoundOdom.getOrigin().getX() + soundOdomToWorldTf.getOrigin().getX();
        ps_groundTruth.y() = groundTruthToSoundOdom.getOrigin().getY() + soundOdomToWorldTf.getOrigin().getY();
        ps_groundTruth.z() = groundTruthToSoundOdom.getOrigin().getZ() + soundOdomToWorldTf.getOrigin().getZ();


        /// Compute the orientation of the robot origin.
        tf::Quaternion q(robotOriginFullToWorldTf.getRotation());
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);


        /// Subscribe the potential sources, which is the directions of the sound.
        // size of potential sources
        int sizeOfPotentialSources = potentialSources_ptr->potential_sources.size();

        /// insert PotentialSourcesPtr to Queue
        tdoa_based_ssl_message::PotentialSourcesAverage tempPotentialSources;

        tempPotentialSources.header.stamp = potentialSources_ptr->header.stamp;
        tempPotentialSources.header.frame_id = potentialSources_ptr->header.frame_id;
        tempPotentialSources.frequency = potentialSources_ptr->frequency;

        float averageProbability = 0;

        for(int i=0 ; i<sizeOfPotentialSources ; i++)
        {            
            tdoa_based_ssl_message::SourceInfoT potentialSourceNode;

            potentialSourceNode.source_id = potentialSources_ptr->potential_sources[i].source_id;
            potentialSourceNode.source_pos.x = potentialSources_ptr->potential_sources[i].source_pos.x;
            potentialSourceNode.source_pos.y = potentialSources_ptr->potential_sources[i].source_pos.y;
            potentialSourceNode.source_pos.z = potentialSources_ptr->potential_sources[i].source_pos.z;
            potentialSourceNode.longitude = potentialSources_ptr->potential_sources[i].longitude;
            potentialSourceNode.latitude = potentialSources_ptr->potential_sources[i].latitude;
            potentialSourceNode.source_probability = potentialSources_ptr->potential_sources[i].source_probability;
            potentialSourceNode.source_energy = potentialSources_ptr->potential_sources[i].source_energy;
            //potentialSourceNode.transform = robotOriginFullToWorldTf;
            potentialSourceNode.robot_pos.x = robotOriginFullToWorldTf.getOrigin().getX();
            potentialSourceNode.robot_pos.y = robotOriginFullToWorldTf.getOrigin().getY();
            potentialSourceNode.robot_pos.z = robotOriginFullToWorldTf.getOrigin().getZ();
            potentialSourceNode.robot_rotate.x = roll;
            potentialSourceNode.robot_rotate.y = pitch;
            potentialSourceNode.robot_rotate.z = yaw;
            potentialSourceNode.groundTruth_pos.x = groundTruthToSoundOdom.getOrigin().getX() + soundOdomToWorldTf.getOrigin().getX();//groundTruthToWorldTf.getOrigin().getX();
            potentialSourceNode.groundTruth_pos.y = groundTruthToSoundOdom.getOrigin().getY() + soundOdomToWorldTf.getOrigin().getY();
            potentialSourceNode.groundTruth_pos.z = groundTruthToSoundOdom.getOrigin().getZ() + soundOdomToWorldTf.getOrigin().getZ();
            averageProbability += potentialSources_ptr->potential_sources[i].source_probability;    // sum of probabilities
            tempPotentialSources.potential_sources.push_back(potentialSourceNode);

            /////////////////////////////////////////////////////////////////////////////           
        }
        tempPotentialSources.average_probability = averageProbability;

        if(ps_potentialSourcesQueue.size() < 500)
        {
            //ps_queueNumber[ps_queueIndex]++;
            ps_numberOfQueue[ps_queueIndex]++;
            ps_potentialSourcesQueue.push_back(tempPotentialSources);
        }

    }

    /// Extract window(60) of Potential Sources from a traditional sound source localization
    void RaybasedSoundlocalization::extractPotentialSourceCallback(const ros::TimerEvent& event)
    {
        /* Calculate a computation time */
        struct timeval start_point, end_point;
        gettimeofday(&start_point, NULL);
        /* Starting point */

        std::cout << " !Extract Potential Source Callback" << std::endl;

        ps_visualParticleToModule.publish(positionToVisualDetector);

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
        topPotentialSources.resize(ps_windowSize);

        for(std::vector<tdoa_based_ssl_message::PotentialSourcesAverage>::iterator it = ps_potentialSourcesQueue.begin()
            ; it != ps_potentialSourcesQueue.end() ; it++) {

            for(int i=0 ; i<it->potential_sources.size() ; i++){
                PotentialSource tempPotentialSource(it->potential_sources[i], it->frequency);
                highIntensityDirection.push(tempPotentialSource);
            }
        }



        ps_soundPotentialField.resetPotentialField(m_octree, ps_filterWindowSize);

        ps_checkGettingPotentialSources = false;    // found the maximum energy window
        if(highIntensityDirection.size() < ps_windowSize){

        }else{

            int rayIndex = 0;

            /// from (maxidx-window size) to (maxidx), default window size : 60
            for(int i=0 ; i<ps_windowSize ; i++){

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

                    ps_checkGettingPotentialSources = true;

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

        /// publish color Marker array to visualize
        publishColorMarker();


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

        /// Compute Generalized Value to determine whether particles are converged or not.
        octomap::point3d targetPos;
        double determinantCov;
        tf::Matrix3x3 covM;
        bool ps_positionDected = ps_particleFilter->getSoundSourcePosition(targetPos, determinantCov, covM);    


        /// resample particles based on weigts.
        if(ps_soundPotentialField.PotentialRays.size() <= 1)
            std::cout << "\t PotentialSources are empty" << std::endl;
        else
            ps_particleFilter->resampleParticles();


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
        std::cout << "Particle filter operation time: " << operation_time << std::endl;
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
                        std_msgs::ColorRGBA _color; _color.r = (r / 255.); _color.g = (g / 255.); _color.b = (b / 255.); _color.a = 1.0; // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
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

            //occupiedRayNodesVis
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

                    unsigned idx = m_maxTreeDepth;
                    assert(idx < occupiedNodesVis.markers.size());

                    geometry_msgs::Point cubeCenter;
                    cubeCenter.x = positionOfRay.x();
                    cubeCenter.y = positionOfRay.y();
                    cubeCenter.z = positionOfRay.z();

                    occupiedRayNodesVis.markers[idx].points.push_back(cubeCenter);

#ifdef COLOR_OCTOMAP_SERVER
                    if (m_useColoredMap) {
                        //std_msgs::ColorRGBA _color; _color.r = (r / 255.); _color.g = (g / 255.); _color.b = (b / 255.); _color.a = 1.0; // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
                        //std_msgs::ColorRGBA _color; _color.r = (r); _color.g = (g); _color.b = (b); _color.a = 1.; // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
                        std_msgs::ColorRGBA color_;

                        float maximumEnergyOfRay = 10;

                        float normalColorValue = (ps_soundPotentialField.PotentialRays[i].ray[j].energy/maximumEnergyOfRay) + 0.2;
                        if(normalColorValue > 1.0) normalColorValue = 1.0;
                        color_.r = normalColorValue;

                        color_.g = 0;
                        color_.b = 0;                        
                        color_.a = 0.3;
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



}   // the end of "raybased_soundlocalization" namespace
