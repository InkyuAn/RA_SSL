#include <ros/ros.h>
#include <rayBasedSSL.h>

#include <iostream>

using namespace raybased_soundlocalization;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "raybased_SoundSourceLocalization");
    const ros::NodeHandle& private_nh = ros::NodeHandle("~");

    ROS_INFO("Let's start rayBased_soundLocalization_node\n");


    RaybasedSoundlocalization server;    

    try{       
        ros::spin();

    }catch(std::runtime_error& e){
        ROS_ERROR("octomap_server exception: %s", e.what());
        return -1;
    }

    return 0;
}
