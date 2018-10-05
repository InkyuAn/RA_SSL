#include <ros/ros.h>

#include <iostream>

#include <octomap/math/Vector3.h>
#include <reflection_aware_ssl_message/visualizeConvergenceParticle.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/message_filter.h>


#define MOVE_DETECTOR 0

ros::Publisher detectorPub[4];
ros::Subscriber pointInfor;
ros::Timer timerSoundDetector;

reflection_aware_ssl_message::visualizeConvergenceParticle insertedPositionDetector;
reflection_aware_ssl_message::visualizeConvergenceParticle pubPositionDetector;
reflection_aware_ssl_message::visualizeConvergenceParticle gotPositionDetector;
octomath::Vector3 gotPosition;
octomath::Vector3 gotScale;
double gotRoll, gotPitch, gotYaw;
reflection_aware_ssl_message::visualizeConvergenceParticle tempPositionOfNULL;


octomath::Vector3 moveVector(0.0f, 0.0f, 0.0f);
octomath::Vector3 scaleVector(0.0f, 0.0f, 0.0f);
double moveRoll, movePitch, moveYaw;


bool menuCase = false;

void insertFromBechmark(const reflection_aware_ssl_message::visualizeConvergenceParticleConstPtr pointInfor_)
{
    //std::cout << "Got data from benchmark ... " << std::endl;
    gotPositionDetector.p_x = pointInfor_->p_x;
    gotPositionDetector.p_y = pointInfor_->p_y;
    gotPositionDetector.p_z = pointInfor_->p_z;
    gotPositionDetector.q_x = pointInfor_->q_x;
    gotPositionDetector.q_y = pointInfor_->q_y;
    gotPositionDetector.q_z = pointInfor_->q_z;
    gotPositionDetector.q_w = pointInfor_->q_w;
    gotPositionDetector.s_x = pointInfor_->s_x;
    gotPositionDetector.s_y = pointInfor_->s_y;
    gotPositionDetector.s_z = pointInfor_->s_z;
    gotPosition.x() = gotPositionDetector.p_x;
    gotPosition.y() = gotPositionDetector.p_y;
    gotPosition.z() = gotPositionDetector.p_z;
    gotScale.x() = gotPositionDetector.s_x;
    gotScale.y() = gotPositionDetector.s_y;
    gotScale.z() = gotPositionDetector.s_z;
    tf::Quaternion gotQuat(pointInfor_->q_x, pointInfor_->q_y, pointInfor_->q_z, pointInfor_->q_w);
    tf::Matrix3x3 m(gotQuat);
    //double roll, pitch, yaw;
    m.getRPY(gotRoll, gotPitch, gotYaw);

    if(pointInfor_->s_x < 0 && pointInfor_->s_y < 0 && pointInfor_->s_z < 0){
        menuCase = false;
        pubPositionDetector = tempPositionOfNULL;
    }else{
        menuCase = true;
        if(pubPositionDetector.s_x < 0 && pubPositionDetector.s_y < 0 && pubPositionDetector.s_z < 0){
            moveVector = octomath::Vector3(0.0f, 0.0f, 0.0f);
            scaleVector = octomath::Vector3(0.0f, 0.0f, 0.0f);
            moveRoll=0.0, movePitch=0.0, moveYaw=0.0;
            pubPositionDetector.p_x = pointInfor_->p_x;
            pubPositionDetector.p_y = pointInfor_->p_y;
            pubPositionDetector.p_z = pointInfor_->p_z;
            pubPositionDetector.q_x = pointInfor_->q_x;
            pubPositionDetector.q_y = pointInfor_->q_y;
            pubPositionDetector.q_z = pointInfor_->q_z;
            pubPositionDetector.q_w = pointInfor_->q_w;
            pubPositionDetector.s_x = pointInfor_->s_x;
            pubPositionDetector.s_y = pointInfor_->s_y;
            pubPositionDetector.s_z = pointInfor_->s_z;
        }else{
            moveVector = octomath::Vector3(pointInfor_->p_x, pointInfor_->p_y, pointInfor_->p_z)
                    - octomath::Vector3(pubPositionDetector.p_x, pubPositionDetector.p_y, pubPositionDetector.p_z);
            scaleVector = octomath::Vector3(pointInfor_->s_x, pointInfor_->s_y, pointInfor_->s_z)
                    - octomath::Vector3(pubPositionDetector.s_x, pubPositionDetector.s_y, pubPositionDetector.s_z);
            tf::Quaternion pubQuat(pubPositionDetector.q_x, pubPositionDetector.q_y, pubPositionDetector.q_z, pubPositionDetector.q_w);
            tf::Matrix3x3 m_(pubQuat);
            double pubRoll, pubPitch, pubYaw;
            m_.getRPY(pubRoll, pubPitch, pubYaw);

            moveRoll = gotRoll-pubRoll, movePitch = gotPitch-pubPitch, moveYaw = gotYaw-pubYaw;
            //moveVector /= (particleTimerPeriod/localTimerPeriod); //25
#if MOVE_DETECTOR==1
            moveVector /= 1.0f;
            scaleVector /= 1.0f;
            moveRoll /= 1.0f; movePitch /= 1.0f; moveYaw /= 1.0f;
#elif MOVE_DETECTOR==0
            moveVector /= 25.0f;
            scaleVector /= 25.0f;
            moveRoll /= 25.0f; movePitch /= 25.0f; moveYaw /= 25.0f;
#endif

            pubPositionDetector.q_x = pointInfor_->q_x;
            pubPositionDetector.q_y = pointInfor_->q_y;
            pubPositionDetector.q_z = pointInfor_->q_z;
            pubPositionDetector.q_w = pointInfor_->q_w;
            //pubPositionDetector.s_x = pointInfor_->s_x;
            //pubPositionDetector.s_y = pointInfor_->s_y;
            //pubPositionDetector.s_z = pointInfor_->s_z;
        }
    }

}

void soundDetectorPubCallback(const ros::TimerEvent& event)
{
    if(menuCase){
        octomath::Vector3 tempPos(pubPositionDetector.p_x, pubPositionDetector.p_y, pubPositionDetector.p_z);
        octomath::Vector3 tempScale(pubPositionDetector.s_x, pubPositionDetector.s_y, pubPositionDetector.s_z);
        if((gotPosition-tempPos).norm() < 0.1){
            pubPositionDetector.p_x = gotPositionDetector.p_x;
            pubPositionDetector.p_y = gotPositionDetector.p_y;
            pubPositionDetector.p_z = gotPositionDetector.p_z;
        }else{
            pubPositionDetector.p_x += moveVector.x();
            pubPositionDetector.p_y += moveVector.y();
            pubPositionDetector.p_z += moveVector.z();
        }
        if((gotScale-tempScale).norm() < 0.1){
            pubPositionDetector.s_x = gotPositionDetector.s_x;
            pubPositionDetector.s_y = gotPositionDetector.s_y;
            pubPositionDetector.s_z = gotPositionDetector.s_z;
        }else{
            pubPositionDetector.s_x += scaleVector.x();
            pubPositionDetector.s_y += scaleVector.y();
            pubPositionDetector.s_z += scaleVector.z();
        }
        tf::Quaternion pubQuat(pubPositionDetector.q_x, pubPositionDetector.q_y, pubPositionDetector.q_z, pubPositionDetector.q_w);
        tf::Matrix3x3 m_(pubQuat);
        double pubRoll, pubPitch, pubYaw;
        m_.getRPY(pubRoll, pubPitch, pubYaw);

        if(octomath::Vector3(gotRoll-pubRoll, gotPitch-pubPitch, gotYaw-pubYaw).norm() < 0.1){
            pubRoll = gotRoll;
            pubPitch = gotPitch;
            pubYaw = gotYaw;
        }else{
            pubRoll += moveRoll;
            pubPitch += movePitch;
            pubYaw += moveYaw;
        }
        m_.setRPY(pubRoll, pubPitch, pubYaw);
        m_.getRotation(pubQuat);
        // Publish Markers of sphere ...
        for(int i=0 ; i<4 ; i++){
            visualization_msgs::Marker sphere_list;
            //sphere_list.markers.resize(3);

            sphere_list.header.stamp = event.current_expected;
            sphere_list.header.frame_id = "map";
            sphere_list.ns = "soundDetector";

            sphere_list.action = visualization_msgs::Marker::ADD;
            sphere_list.id = 0;
            sphere_list.type = visualization_msgs::Marker::SPHERE;

            sphere_list.scale.x = pubPositionDetector.s_x*(i+1);
            sphere_list.scale.y = pubPositionDetector.s_y*(i+1);
            sphere_list.scale.z = pubPositionDetector.s_z*(i+1);


            //sphere_list.markers[i].color.r = 1.0f; sphere_list.markers[i].color.a = 1.0 - i*0.25;
            if(i==0){
                sphere_list.color.r = 1.0f; sphere_list.color.g = 1.0f; sphere_list.color.a = 0.7;
            }else if(i==1){
                sphere_list.color.r = 1.0f; sphere_list.color.g = 1.0f; sphere_list.color.a = 0.5;
            }else if(i==2){
                sphere_list.color.r = 1.0f; sphere_list.color.g = 1.0f; sphere_list.color.a = 0.3;
            }else{
                sphere_list.color.r = 1.0f; sphere_list.color.g = 1.0f; sphere_list.color.a = 0.15;
            }            



            sphere_list.pose.position.x = pubPositionDetector.p_x;
            sphere_list.pose.position.y = pubPositionDetector.p_y;
            sphere_list.pose.position.z = pubPositionDetector.p_z;

            //sphere_list.pose.orientation.x = pubPositionDetector.q_x;
            //sphere_list.pose.orientation.y = pubPositionDetector.q_y;
            //sphere_list.pose.orientation.z = pubPositionDetector.q_z;
            //sphere_list.pose.orientation.w = pubPositionDetector.q_w;
            sphere_list.pose.orientation.x = pubQuat.x();
            sphere_list.pose.orientation.y = pubQuat.y();
            sphere_list.pose.orientation.z = pubQuat.z();
            sphere_list.pose.orientation.w = pubQuat.y();

            detectorPub[i].publish(sphere_list);
        }
    }else{
        for(int i=0 ; i<4 ; i++){
            visualization_msgs::Marker sphere_list;
            //sphere_list.markers.resize(3);

            sphere_list.header.stamp = event.current_expected;
            sphere_list.header.frame_id = "map";
            sphere_list.ns = "soundDetector";

            sphere_list.action = visualization_msgs::Marker::ADD;
            sphere_list.id = 0;
            sphere_list.type = visualization_msgs::Marker::SPHERE;

            sphere_list.scale.x = 1;
            sphere_list.scale.y = 1;
            sphere_list.scale.z = 1;

            sphere_list.color.a = 0.0f;
            //sphere_list.markers[i].color.r = 1.0f; sphere_list.markers[i].color.a = 1.0 - i*0.25;
            sphere_list.pose.position.x = 0;
            sphere_list.pose.position.y = 0;
            sphere_list.pose.position.z = 0;

            detectorPub[i].publish(sphere_list);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "raybased_ssl_detector");
    const ros::NodeHandle& private_nh = ros::NodeHandle("~");
    ros::NodeHandle m_nh = ros::NodeHandle("~");

#if MOVE_DETECTOR == 0
    detectorPub[0] = m_nh.advertise<visualization_msgs::Marker>("raybased_detector1", 1, true);
    detectorPub[1] = m_nh.advertise<visualization_msgs::Marker>("raybased_detector2", 1, true);
    detectorPub[2] = m_nh.advertise<visualization_msgs::Marker>("raybased_detector3", 1, true);
    detectorPub[3] = m_nh.advertise<visualization_msgs::Marker>("raybased_detector4", 1, true);
#elif MOVE_DETECTOR == 1
    detectorPub[0] = m_nh.advertise<visualization_msgs::MarkerArray>("raybased_detector1", 1, true);
    detectorPub[1] = m_nh.advertise<visualization_msgs::MarkerArray>("raybased_detector2", 1, true);
    detectorPub[2] = m_nh.advertise<visualization_msgs::MarkerArray>("raybased_detector3", 1, true);
    detectorPub[3] = m_nh.advertise<visualization_msgs::MarkerArray>("raybased_detector4", 1, true);
#endif



    pointInfor = m_nh.subscribe("/rassl_to_visualDetector", 1, &insertFromBechmark);

    tempPositionOfNULL.s_x = -1.0f;
    tempPositionOfNULL.s_y = -1.0f;
    tempPositionOfNULL.s_z = -1.0f;

    pubPositionDetector.s_x = -1.0f;
    pubPositionDetector.s_y = -1.0f;
    pubPositionDetector.s_z = -1.0f;

    timerSoundDetector = m_nh.createTimer(ros::Duration(0.02), &soundDetectorPubCallback);

    ROS_INFO("Let's start rayBased benchmark visual Detector\n");


    try{
        //ROS_INFO("Tried ros::spin\n");
        ros::spin();

    }catch(std::runtime_error& e){
        ROS_ERROR("octomap_server exception: %s", e.what());
        return -1;
    }

    return 0;
}
