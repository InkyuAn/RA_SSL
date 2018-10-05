#ifndef RAYBASED_SOUNDLOCALIZATION_H
#define RAYBASED_SOUNDLOCALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <eigen3/Eigen/Eigenvalues>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_server/OctomapServerConfig.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>

/// Potential source
#include "tdoa_ra_ssl_message/PotentialSources.h"
#include "tdoa_ra_ssl_message/PotentialSourcesAverage.h"
#include "tdoa_ra_ssl_message/SourceInfoT.h"
#include "tdoa_ra_ssl_message/sourceGroundTruth.h"

#include "reflection_aware_ssl_message/visualizeConvergenceParticle.h"

#include <soundPotentialField.h>

#include <sys/time.h>
#include <queue>

#define COLOR_OCTOMAP_SERVER

#ifdef COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif


namespace raybased_soundlocalization{

    /// Data type of Potentail source, which denotes the direciton of sound
    class PotentialSource{
    public:
        PotentialSource(tdoa_ra_ssl_message::SourceInfoT node_, float freq_){
            frequency = freq_;
            potentialSourceNode.source_id = node_.source_id;
            potentialSourceNode.source_pos.x = node_.source_pos.x;
            potentialSourceNode.source_pos.y = node_.source_pos.y;
            potentialSourceNode.source_pos.z = node_.source_pos.z;
            potentialSourceNode.longitude = node_.longitude;
            potentialSourceNode.latitude = node_.latitude;
            potentialSourceNode.source_probability = node_.source_probability;
            potentialSourceNode.source_energy = node_.source_energy;
            //potentialSourceNode.transform = robotOriginFullToWorldTf;
            potentialSourceNode.robot_pos.x = node_.robot_pos.x;
            potentialSourceNode.robot_pos.y = node_.robot_pos.y;
            potentialSourceNode.robot_pos.z = node_.robot_pos.z;
            potentialSourceNode.robot_rotate.x = node_.robot_rotate.x;
            potentialSourceNode.robot_rotate.y = node_.robot_rotate.y;
            potentialSourceNode.robot_rotate.z = node_.robot_rotate.z;
            potentialSourceNode.groundTruth_pos.x = node_.groundTruth_pos.x;
            potentialSourceNode.groundTruth_pos.y = node_.groundTruth_pos.y;
            potentialSourceNode.groundTruth_pos.z = node_.groundTruth_pos.z;
        }
        PotentialSource(){
            frequency = 0;
            potentialSourceNode.source_id = 0;
            potentialSourceNode.source_pos.x = 0;
            potentialSourceNode.source_pos.y = 0;
            potentialSourceNode.source_pos.z = 0;
            potentialSourceNode.longitude = 0;
            potentialSourceNode.latitude = 0;
            potentialSourceNode.source_probability = 0;
            potentialSourceNode.source_energy = 0;
            //potentialSourceNode.transform = robotOriginFullToWorldTf;
            potentialSourceNode.robot_pos.x = 0;
            potentialSourceNode.robot_pos.y = 0;
            potentialSourceNode.robot_pos.z = 0;
            potentialSourceNode.robot_rotate.x = 0;
            potentialSourceNode.robot_rotate.y = 0;
            potentialSourceNode.robot_rotate.z = 0;
            potentialSourceNode.groundTruth_pos.x = 0;
            potentialSourceNode.groundTruth_pos.y = 0;
            potentialSourceNode.groundTruth_pos.z = 0;
        }

        ~PotentialSource(){

        }

        tdoa_ra_ssl_message::SourceInfoT potentialSourceNode;
        float frequency;
    };

    class RaybasedSoundlocalization{
    public:

#ifdef COLOR_OCTOMAP_SERVER
        typedef octomap::ColorOcTree OcTreeT;
#else
        typedef octomap::OcTree OcTreeT;
#endif
        typedef octomap_msgs::GetOctomap OctomapSrv;
        typedef octomap_msgs::BoundingBoxQuery BBXSrv;

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

        double PI;

        RaybasedSoundlocalization(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));

        virtual ~RaybasedSoundlocalization();

        virtual void insertSourcePositionCallback(const tdoa_ra_ssl_message::sourceGroundTruthConstPtr& sourcePosPtr);

        virtual void publishColorMarker(const ros::Time& rostime = ros::Time::now());

        virtual void publishCylinders(octomap::point3d inputGroundTruth, octomap::point3d inputRobotPos);

        virtual void insertPotentialsourcesCallback(const tdoa_ra_ssl_message::PotentialSourcesConstPtr& potentialSources_ptr);

        virtual void extractPotentialSourceCallback(const ros::TimerEvent&);

        virtual void particleFilterCallback(const ros::TimerEvent& event);


    protected:
        /// Project path
        /// You have to modify this for your working environment.
        std::string projectPath;

        ///
        ros::NodeHandle m_nh;

        /// Publishers
        ros::Publisher m_markerPub;                
        ros::Publisher m_rayMarkerPub;        

        ros::Publisher ps_particlePub;
        ros::Publisher robotPosPublisher;
        ros::Publisher groundTruthPosPublisher;
        ros::Publisher ps_visualParticleToModule;

        bool m_latchedTopics;

        /// Timers
        ros::Timer particleFilter_timer;
        ros::Timer extractPS_timer;
        ros::Timer controller_timer;

        /// Subscribers
        ros::Subscriber ps_potentialSourcesSub;
        ros::Subscriber ps_sourcePositionSubscriber;


        /// Octo map
        OcTreeT* m_octree;

        std::string m_worldFrameId; // the map frame

        octomap::point3d ps_soundCubeToBaseFootprint;

        double m_res;
        unsigned m_treeDepth;
        unsigned m_maxTreeDepth;
        bool m_useColoredMap;

        std_msgs::ColorRGBA m_color;
        std_msgs::ColorRGBA m_colorFree;        

        bool ps_drawSoundPotentialField;

        double ps_minLatitude;
        double ps_maxLatitude;
        float ps_minEnergy;

        std::vector<tdoa_ra_ssl_message::PotentialSourcesAverage> ps_potentialSourcesQueue;

        /// source position (ground truth)
        float groundTruthPosition[3];

        int ps_numberOfQueue[4];
        unsigned ps_queueIndex;
        double ps_ceilingHeight;    /// Ceilling height ( meter )

        /// These variables are used for the propagation energy of rays.
        double ps_temperature;
        double ps_humidity;
        soundAttenuation atmosphericAbsorption;

        /// Sound Potential Field, which contains the voxel based acoustic rays.
        SoundPotentialField ps_soundPotentialField;

        /// For Particle filter
        ParticleFilter* ps_particleFilter;       
        int ps_numberOfParticle;


        /// For visualizing estimated positions of the source
        reflection_aware_ssl_message::visualizeConvergenceParticle positionToVisualDetector;        


    };
}



#endif
