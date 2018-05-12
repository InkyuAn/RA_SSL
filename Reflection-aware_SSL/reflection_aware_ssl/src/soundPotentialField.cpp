#include <soundPotentialField.h>

namespace raybased_soundlocalization{

    //SoundPotentialField
    SoundPotentialField::SoundPotentialField()
    {
        double pi = 3.14159265359;
        double squreTwoOverTwo = std::sqrt(2)/2.0;
        MCparam_twoPointfive = (1.0/72.0)*pi;
        MCparam_onePointtwofive = (1.0/144.0)*pi;
        MCparam_half_twoPointfive = MCparam_twoPointfive * squreTwoOverTwo;
        MCparam_half_onePointtwofive = MCparam_onePointtwofive * squreTwoOverTwo;
        std::cout << "MCparam_twoPointfive, MCparam_onePointtwofive: " << MCparam_twoPointfive << " " << MCparam_onePointtwofive << std::endl;
        std::cout << "MCparam_half_twoPointfive, MCparam_half_onePointtwofive: " << MCparam_half_twoPointfive << " " << MCparam_half_onePointtwofive << std::endl;

    }

    SoundPotentialField::~SoundPotentialField()
    {

        //resetPotentialField(width, height);

        PotentialRays.clear();
    }

    octomap::point3d SoundPotentialField::sphericalCoorToCartesian(octomap::point3d sphericalCoor)
    {
        octomap::point3d cartesianCoor;
        cartesianCoor.x() = std::cos(sphericalCoor.y()) * std::sin(sphericalCoor.z()) * sphericalCoor.x();
        cartesianCoor.y() = std::sin(sphericalCoor.y()) * std::sin(sphericalCoor.z()) * sphericalCoor.x();
        cartesianCoor.z() = std::cos(sphericalCoor.z()) * sphericalCoor.x();
        return cartesianCoor;
    }

    octomap::point3d SoundPotentialField::cartesianToSphericalcoor(octomap::point3d cartesianCoor)
    {
        octomap::point3d sphericalCoor;
        sphericalCoor.x() = std::sqrt(cartesianCoor.x()*cartesianCoor.x() + cartesianCoor.y()*cartesianCoor.y() + cartesianCoor.z()*cartesianCoor.z());
        sphericalCoor.y() = std::atan2(cartesianCoor.y(), cartesianCoor.x());
        sphericalCoor.z() = std::acos(cartesianCoor.z()/sphericalCoor.x());
        return sphericalCoor;
    }

    void SoundPotentialField::initPotentialField(double ps_ceilingHeight, double ps_bottomHeight, unsigned ps_depthGridKeyMap, float ps_filterWindowSize)
    {
        ceilingHeight = ps_ceilingHeight;
        bottomHeight = ps_bottomHeight;
        depthGridKeyMap = ps_depthGridKeyMap;

        filterWindowSize = ps_filterWindowSize;
    }

    void SoundPotentialField::resetPotentialField(octomap::ColorOcTree* octree, float ps_filterWindowSize)
    {
        filterWindowSize = ps_filterWindowSize;
        // reset gridmap
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

        //octomap::OcTreeKey paddedMaxKey, paddedMinKey;
        unsigned treeDepth = octree->getTreeDepth();

        gridMapMinKey = octree->coordToKey(minPt, treeDepth);
        gridMapMaxKey = octree->coordToKey(maxPt, treeDepth);

        // Set filter size
        filterWindowKeySize = int(filterWindowSize / octree->getResolution());
        if((filterWindowKeySize%2) == 1)
            filterWindowKeySize+=1;
        HalfFilterWindowKeySize = int(filterWindowKeySize*0.5);

        // clear PotentialRay Vector
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

    void SoundPotentialField::traverseMonteCarloBasedSoundRay(float initEnergy, const octomap::point3d& originP, const octomap::point3d& directionP,
                                                              double maxRange, octomap::ColorOcTree* octree, int rayIndex)
    {
        octomap::point3d monteCarlo_SPdirections[17];
        octomap::point3d SPdirection = cartesianToSphericalcoor(directionP);

        monteCarlo_SPdirections[0]  = SPdirection + octomap::point3d(0.0, 0.0, 0.0);
        monteCarlo_SPdirections[1]  = SPdirection + octomap::point3d(0.0, MCparam_twoPointfive, 0.0);
        monteCarlo_SPdirections[2]  = SPdirection + octomap::point3d(0.0, MCparam_onePointtwofive, 0.0);
        monteCarlo_SPdirections[3]  = SPdirection + octomap::point3d(0.0, 0.0, MCparam_twoPointfive);
        monteCarlo_SPdirections[4]  = SPdirection + octomap::point3d(0.0, 0.0, MCparam_onePointtwofive);
        monteCarlo_SPdirections[5]  = SPdirection + octomap::point3d(0.0, MCparam_twoPointfive*(-1), 0.0);
        monteCarlo_SPdirections[6]  = SPdirection + octomap::point3d(0.0, MCparam_onePointtwofive*(-1), 0.0);
        monteCarlo_SPdirections[7]  = SPdirection + octomap::point3d(0.0, 0.0, MCparam_twoPointfive*(-1));
        monteCarlo_SPdirections[8]  = SPdirection + octomap::point3d(0.0, 0.0, MCparam_onePointtwofive*(-1));
        monteCarlo_SPdirections[9]  = SPdirection + octomap::point3d(0.0, MCparam_half_twoPointfive, MCparam_half_twoPointfive);
        monteCarlo_SPdirections[10] = SPdirection + octomap::point3d(0.0, MCparam_half_twoPointfive, MCparam_half_twoPointfive*(-1));
        monteCarlo_SPdirections[11] = SPdirection + octomap::point3d(0.0, MCparam_half_twoPointfive*(-1), MCparam_half_twoPointfive);
        monteCarlo_SPdirections[12] = SPdirection + octomap::point3d(0.0, MCparam_half_twoPointfive*(-1), MCparam_half_twoPointfive*(-1));
        monteCarlo_SPdirections[13] = SPdirection + octomap::point3d(0.0, MCparam_half_onePointtwofive, MCparam_half_onePointtwofive);
        monteCarlo_SPdirections[14] = SPdirection + octomap::point3d(0.0, MCparam_half_onePointtwofive, MCparam_half_onePointtwofive*(-1));
        monteCarlo_SPdirections[15] = SPdirection + octomap::point3d(0.0, MCparam_half_onePointtwofive*(-1), MCparam_half_onePointtwofive);
        monteCarlo_SPdirections[16] = SPdirection + octomap::point3d(0.0, MCparam_half_onePointtwofive*(-1), MCparam_half_onePointtwofive*(-1));

        soundRaytracing::KeyRayEnergy ray_(initEnergy, gridMapMaxKey.k[2], gridMapMinKey.k[2], rayIndex);

        for(int i=0 ; i<17 ; i++)
        {
            //std::cout << "monteCarlo_SPdirections: " << monteCarlo_SPdirections[i].x() << " " << monteCarlo_SPdirections[i].y() << " " << monteCarlo_SPdirections[i].z() << std::endl;
            octomap::point3d monteCarlo_direction = sphericalCoorToCartesian(monteCarlo_SPdirections[i]);
            //std::cout << "monteCarlo_direction: " << monteCarlo_direction.x() << " " << monteCarlo_direction.y() << " " << monteCarlo_direction.z() << std::endl;

            ray_.computeEnergyRayKeys(octree, originP, monteCarlo_direction, maxRange);
        }

        PotentialRays.push_back(ray_);
    }
    void SoundPotentialField::traverseMonteCarloBasedSoundRay(float initEnergy, const octomap::point3d& originP, const octomap::point3d& directionP,
                                                              double maxRange, octomap::ColorOcTree* octree, int rayIndex,
                                                              raybased_soundlocalization::soundAttenuation atmosphericAbsorption, float freq)
    {
        octomap::point3d monteCarlo_SPdirections[17];
        octomap::point3d SPdirection = cartesianToSphericalcoor(directionP);

        monteCarlo_SPdirections[0]  = SPdirection + octomap::point3d(0.0, 0.0, 0.0);
        monteCarlo_SPdirections[1]  = SPdirection + octomap::point3d(0.0, MCparam_twoPointfive, 0.0);
        monteCarlo_SPdirections[2]  = SPdirection + octomap::point3d(0.0, MCparam_onePointtwofive, 0.0);
        monteCarlo_SPdirections[3]  = SPdirection + octomap::point3d(0.0, 0.0, MCparam_twoPointfive);
        monteCarlo_SPdirections[4]  = SPdirection + octomap::point3d(0.0, 0.0, MCparam_onePointtwofive);
        monteCarlo_SPdirections[5]  = SPdirection + octomap::point3d(0.0, MCparam_twoPointfive*(-1), 0.0);
        monteCarlo_SPdirections[6]  = SPdirection + octomap::point3d(0.0, MCparam_onePointtwofive*(-1), 0.0);
        monteCarlo_SPdirections[7]  = SPdirection + octomap::point3d(0.0, 0.0, MCparam_twoPointfive*(-1));
        monteCarlo_SPdirections[8]  = SPdirection + octomap::point3d(0.0, 0.0, MCparam_onePointtwofive*(-1));
        monteCarlo_SPdirections[9]  = SPdirection + octomap::point3d(0.0, MCparam_half_twoPointfive, MCparam_half_twoPointfive);
        monteCarlo_SPdirections[10] = SPdirection + octomap::point3d(0.0, MCparam_half_twoPointfive, MCparam_half_twoPointfive*(-1));
        monteCarlo_SPdirections[11] = SPdirection + octomap::point3d(0.0, MCparam_half_twoPointfive*(-1), MCparam_half_twoPointfive);
        monteCarlo_SPdirections[12] = SPdirection + octomap::point3d(0.0, MCparam_half_twoPointfive*(-1), MCparam_half_twoPointfive*(-1));
        monteCarlo_SPdirections[13] = SPdirection + octomap::point3d(0.0, MCparam_half_onePointtwofive, MCparam_half_onePointtwofive);
        monteCarlo_SPdirections[14] = SPdirection + octomap::point3d(0.0, MCparam_half_onePointtwofive, MCparam_half_onePointtwofive*(-1));
        monteCarlo_SPdirections[15] = SPdirection + octomap::point3d(0.0, MCparam_half_onePointtwofive*(-1), MCparam_half_onePointtwofive);
        monteCarlo_SPdirections[16] = SPdirection + octomap::point3d(0.0, MCparam_half_onePointtwofive*(-1), MCparam_half_onePointtwofive*(-1));

        soundRaytracing::KeyRayEnergy ray_(initEnergy, gridMapMaxKey.k[2], gridMapMinKey.k[2], rayIndex);
        ray_.init(atmosphericAbsorption, freq);

        for(int i=0 ; i<17 ; i++)
        {
            //std::cout << "monteCarlo_SPdirections: " << monteCarlo_SPdirections[i].x() << " " << monteCarlo_SPdirections[i].y() << " " << monteCarlo_SPdirections[i].z() << std::endl;
            octomap::point3d monteCarlo_direction = sphericalCoorToCartesian(monteCarlo_SPdirections[i]);
            //std::cout << "monteCarlo_direction: " << monteCarlo_direction.x() << " " << monteCarlo_direction.y() << " " << monteCarlo_direction.z() << std::endl;

            ray_.computeEnergyRayKeys(octree, originP, monteCarlo_direction, maxRange);
        }
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
