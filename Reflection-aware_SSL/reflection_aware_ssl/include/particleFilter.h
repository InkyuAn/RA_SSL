#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <octomap/ColorOcTree.h>
#include <acousticRaytracing.h>

#include <iostream>
#include <time.h>       /* time */
#include <random>
#include <algorithm>
#include <pcl_ros/point_cloud.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace raybased_soundlocalization{

    class particle{
    public:
        particle(octomap::point3d p, int i){
            position = p;
            weight = 0;
            numOfPreweit = 10;
            prevweight.resize(numOfPreweit, 0.0f);        
            index = i;
        }
        particle(float x, float y, float z, int i){
            position.x() = x;
            position.y() = y;
            position.z() = z;
            numOfPreweit = 7;
            prevweight.resize(numOfPreweit, 0.0f);           
            weight = 0;
            index = i;
        }
        void resetParticle() {
            weight = 0;
            for (int i=0 ; i<numOfPreweit ; i++){
                prevweight[i] = 0.0f;
            }
        }
        void reduceWeight() {           
            float sumOfPreWeight = 0.0f;

            for(int i=numOfPreweit-1 ; i>0 ; i--){
                prevweight[i] = prevweight[i-1]*0.3;
                sumOfPreWeight += prevweight[i];
            }
            if(numOfPreweit != 0)
                prevweight[0] = weight * 0.3;                
            sumOfPreWeight += prevweight[0];

            weight = sumOfPreWeight;
        }

        void addWeight(float w) {
            weight += w;
        }


        octomap::point3d position;
        float weight;
        std::vector<float> prevweight;
        int numOfPreweit;
        int index;

        bool operator <(const particle &a) const
        {
            return this->weight < a.weight;
        }
    };

    class ParticleFilter{
    public:
        ParticleFilter(int numberOfSample_, int numberOfArrivalSample_, float varOfWeight_, float varOfSoundModel_, octomap::point3d MaxMap_, octomap::point3d MinMap_, float minEnergy);
        ~ParticleFilter();

        void setOctreeInf(octomap::ColorOcTree* Octree_){
            Octree = Octree_;
        }

        void initParticleFilter(int particlesNumber);
        void resetParticles();
        void reInitParticleFilter(octomap::point3d MaxMap_, octomap::point3d MinMap_);
        void resetWeightOfParticles();
        float computeWeightOfPaticle(float distance, float distanceOfRayToIntersectedPoint, float energy, float initEnergy);

        float computeDistancePointToLines(octomap::point3d point, soundRaytracing::KeyRayEnergy& line, float &rayDistance, octomap::point3d& intersectedPoint, int j);

        bool computeDistancePointToLine(octomap::point3d point, octomap::point3d origin, octomap::point3d end, float& distance,octomap::point3d& intersectedPoint);
        void computeImportanceSamplingWeight(std::vector<soundRaytracing::KeyRayEnergy>& rays);    // w_i
        void predictNextPositionOfParticle();  // p( x_t | x_(t-1) )
        void resampleParticles();

        void printParticlePosition();
        void printParticleWeight();

        void transferParticleToPointCloud(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &pointClouds);

        bool getSoundSourcePosition(octomap::point3d& goalPosition, double& determinantCov, tf::Matrix3x3& covM);

        void setRobotOrigin(octomap::point3d origin_);
        float minimumEnergy;
    private:
        /* function */
        float getUniformRandomValue(float min, float max);
        float getGaussianRandomValue(float mean, float var);
        float getGaussianDistribution(float dist);

        /* variable */
        float PI;
        int numberOfParticles;
        int numberOfArrivalParticles;
        float varOfWeight;
        float varOfSoundModel;
        float sumOfWeight;
        octomap::point3d maxMap;
        octomap::point3d minMap;
        octomap::point3d mapSize;

        octomap::point3d robotOrigin;
        std::vector<particle> particles;

        octomap::point3d meanOfParticles;
        octomap::point3d varianceOfParticles;
        tf::Matrix3x3 covarianceOfParticles;

        bool isSoundDetected;
        bool isAcousticRay;

        octomap::ColorOcTree* Octree;
    };
}





#endif
