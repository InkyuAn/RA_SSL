#include <soundPotentialField.h>

namespace raybased_soundlocalization{

    //SoundPotentialField
    SoundPotentialField::SoundPotentialField()
    {
    }

    SoundPotentialField::~SoundPotentialField()
    {
        PotentialRays.clear();
    }

    void SoundPotentialField::initPotentialField(double ps_ceilingHeight, double ps_bottomHeight, unsigned ps_depthGridKeyMap)
    {
        ceilingHeight = ps_ceilingHeight;
        bottomHeight = ps_bottomHeight;
        depthGridKeyMap = ps_depthGridKeyMap;
    }

    void SoundPotentialField::resetPotentialField(octomap::ColorOcTree* octree)
    {
        /// reset gridmap
        if(octree == NULL)
            return;

        double minX, minY, minZ, maxX, maxY, maxZ;
        octree->getMetricMin(minX, minY, minZ);
        octree->getMetricMax(maxX, maxY, maxZ);

        minX = std::abs(minX);
        minY = std::abs(minY);
        maxX = std::abs(maxX);
        maxY = std::abs(maxY);

        double x_ = (maxX>minX)?maxX:minX;
        double y_ = (maxY>minY)?maxY:minY;

        octomap::point3d minPt(x_*-1.0, y_*-1.0, bottomHeight);
        octomap::point3d maxPt(x_, y_, ceilingHeight);

        unsigned treeDepth = octree->getTreeDepth();

        gridMapMinKey = octree->coordToKey(minPt, treeDepth);
        gridMapMaxKey = octree->coordToKey(maxPt, treeDepth);

        /// clear PotentialRay Vector
        if(!PotentialRays.empty())
            PotentialRays.clear();        
    }

    void SoundPotentialField::traverseSoundRay(float initEnergy, const octomap::point3d& originP, const octomap::point3d& directionP, double maxRange, octomap::ColorOcTree* octree, int rayIndex)
    {
        soundRaytracing::KeyRayEnergy ray_(initEnergy, gridMapMaxKey.k[2], gridMapMinKey.k[2], rayIndex);

        ray_.computeEnergyRayKeys(octree, originP, directionP, maxRange);

        PotentialRays.push_back(ray_);

    }

    // This function is used
    void SoundPotentialField::traverseSoundRay(float initEnergy, const octomap::point3d& originP, const octomap::point3d& directionP,
                                               double maxRange, octomap::ColorOcTree* octree, int rayIndex,
                                               raybased_soundlocalization::soundAttenuation atmosphericAbsorption, float freq)
    {
        soundRaytracing::KeyRayEnergy ray_(initEnergy, gridMapMaxKey.k[2], gridMapMinKey.k[2], rayIndex);

        ray_.init(atmosphericAbsorption, freq);

        ray_.computeEnergyRayKeys(octree, originP, directionP, maxRange);


        PotentialRays.push_back(ray_);
    }

    void SoundPotentialField::setNumberOfRays(int number_)
    {
        numberOfRays = number_;
    }

    int SoundPotentialField::getNumberOfRays()
    {
        return numberOfRays;
    }

    bool SoundPotentialField::isPotentialRaysEmpty()
    {
        return PotentialRays.empty();
    }

    SoundPotentialField::iterator SoundPotentialField::getPotentialRayBack()
    {
        return (PotentialRays.end() - 1);
    }

    void SoundPotentialField::popPotentialRayBack()
    {
        PotentialRays.pop_back();
        return;
    }




}
