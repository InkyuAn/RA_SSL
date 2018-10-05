#ifndef SOUND_POTENTIAL_FIELD_H
#define SOUND_POTENTIAL_FIELD_H

#include <fstream>

#include <acousticRaytracing.h>
#include <visualization_msgs/MarkerArray.h>
//#include <time.h>
#include <particleFilter.h>

using namespace soundRaytracing;

namespace raybased_soundlocalization {

    class SoundPotentialField{
    private:

        unsigned depthGridKeyMap;

        double ceilingHeight;
        double bottomHeight;

        octomap::OcTreeKey gridMapMinKey;   // (x), (y), (z:bottom height)
        octomap::OcTreeKey gridMapMaxKey;   // (x), (y), (z:ceiling height)


        int numberOfRays;

    public:
        std::vector<KeyRayEnergy> PotentialRays;
        typedef std::vector<KeyRayEnergy>::iterator iterator;

        /* function */

        SoundPotentialField();
        ~SoundPotentialField();

        void initPotentialField(double ps_ceilingHeight, double ps_bottomHeight, unsigned ps_depthGridKeyMap);
        void resetPotentialField(octomap::ColorOcTree* octree);
        void traverseSoundRay(float initEnergy, const octomap::point3d& originP, const octomap::point3d& directionP, double maxRange, octomap::ColorOcTree* octomap, int rayIndex);
        void traverseSoundRay(float initEnergy, const octomap::point3d& originP, const octomap::point3d& directionP, double maxRange, octomap::ColorOcTree* octomap, int rayIndex, raybased_soundlocalization::soundAttenuation atmosphericAbsorption, float freq);


        /// Acoustic ray's functions
        void setNumberOfRays(int number_);
        int getNumberOfRays();
        bool isPotentialRaysEmpty();
        iterator getPotentialRayBack();
        void popPotentialRayBack();


    };

}

#endif
