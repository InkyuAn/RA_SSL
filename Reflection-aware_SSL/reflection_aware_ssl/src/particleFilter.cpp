#include <particleFilter.h>

namespace raybased_soundlocalization{

    ParticleFilter::ParticleFilter(int numberOfSample_, int numberOfArrivalSample_, float varOfWeight_, float varOfSoundModel_, octomap::point3d MaxMap_, octomap::point3d MinMap_, float minEnergy)
        : PI(3.14159f), sumOfWeight(0.0f), meanOfParticles(0.0f, 0.0f, 0.0f), varianceOfParticles(0.0f, 0.0f, 0.0f), isSoundDetected(false), isAcousticRay(false),
          covarianceOfParticles(0.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f)
    {
        minimumEnergy = minEnergy;
        //numberOfParticles = numberOfSample_;
        numberOfArrivalParticles = numberOfArrivalSample_;
        varOfWeight = varOfWeight_;
        varOfSoundModel = varOfSoundModel_;
        maxMap = MaxMap_;
        minMap = MinMap_;
        robotOrigin = octomap::point3d(0.0f, 0.0f, 0.0f);
        mapSize = maxMap - minMap + octomap::point3d(1.0, 1.0, 1.0);
        Octree = NULL;

        initParticleFilter(numberOfSample_);
    }

    ParticleFilter::~ParticleFilter()
    {

    }

    void ParticleFilter::setRobotOrigin(octomap::point3d origin_)
    {
        robotOrigin = origin_;
    }

    /// Uniform sampling
    float ParticleFilter::getUniformRandomValue(float min, float max)
    {
        std::random_device rn;
        std::mt19937_64 rnd(rn());

        std::uniform_real_distribution<float> unif(min, max);

        return unif(rnd);
    }

    /// Get gaussian random variable
    float ParticleFilter::getGaussianRandomValue(float mean, float var)
    {
        std::random_device rn;
        std::mt19937_64 rnd(rn());

        std::normal_distribution<float> gaussian(mean, var);

        return gaussian(rnd);

    }

    float ParticleFilter::getGaussianDistribution(float dist)
    {
        return (1/std::sqrt(2*PI*varOfWeight))*std::exp((dist*dist*-1) / (2*varOfWeight));
    }

    void ParticleFilter::printParticlePosition()
    {
        std::cout << "print Particle's positions" << std::endl;

        for(int i=0 ; i<numberOfParticles ; i++)
        {
            std::cout << "x, y, z [" << i << "]: " << (particles.begin()+i)->position.x() << " " << (particles.begin()+i)->position.y() << " " << (particles.begin()+i)->position.z() << std::endl;
        }
    }

    void ParticleFilter::printParticleWeight()
    {
        std::cout << "print Particle's weights" << std::endl;

        for(int i=0 ; i<numberOfParticles ; i++)
        {
            std::cout << "weight[" << i << "]: " << (particles.begin()+i)->weight<< std::endl;
        }
    }

    void ParticleFilter::initParticleFilter(int particlesNumber)
    {
        numberOfParticles = particlesNumber;
        //srand((unsigned int)time(NULL));

        for(int i=0 ; i<numberOfParticles ; i++)
        {
            double x = getUniformRandomValue(minMap.x(), maxMap.x());
            double y = getUniformRandomValue(minMap.y(), maxMap.y());
            double z = getUniformRandomValue(minMap.z(), maxMap.z());

            particle particle_(x , y , z , 0.0);

            particles.push_back(particle_);
        }
    }

    void ParticleFilter::resetParticles()
    {
        for(int i=0 ; i<numberOfParticles ; i++){
            double x = getUniformRandomValue(minMap.x(), maxMap.x());
            double y = getUniformRandomValue(minMap.y(), maxMap.y());
            double z = getUniformRandomValue(minMap.z(), maxMap.z());

            (particles.begin()+i)->position.x() = x;
            (particles.begin()+i)->position.y() = y;
            (particles.begin()+i)->position.z() = z;
            (particles.begin()+i)->resetParticle();
        }
    }

    void ParticleFilter::reInitParticleFilter(octomap::point3d MaxMap_, octomap::point3d MinMap_)
    {
        maxMap = MaxMap_;
        minMap = MinMap_;
        mapSize = maxMap - minMap + octomap::point3d(1.0, 1.0, 1.0);
    }

    void ParticleFilter::resetWeightOfParticles()
    {
        for(int i=0 ; i<numberOfParticles ; i++)
        {            
            (particles.begin()+i)->reduceWeight();
        }
    }

    float ParticleFilter::computeWeightOfPaticle(float distance, float distanceOfRayToIntersectedPoint, float energy, float initEnergy)
    {
        /// Propagated ray energy is not used at this time.
        return getGaussianDistribution(distance);

    }

    /// Copmute the distance weight of particles.
    float ParticleFilter::computeDistancePointToLines(octomap::point3d point, soundRaytracing::KeyRayEnergy& line, float &rayDistance, octomap::point3d& intersectedPoint_, int j)  // with reflection
    {        
        float maxWeightOfParticle = 0.0f;        
        std::vector<float> fullDistance;
        fullDistance.resize(line.endPoints.size()-1, 0.0f);
        int checkingIndex = 0;

        rayDistance = 999999.9f;
        intersectedPoint_ = octomap::point3d(999999.9f, 999999.9f, 999999.9f);        

        for(int i = 0 ; i<line.endPoints.size()-1 ; i++)
        {
            //std::cout << "endPoint: " << line.endPoints[i] << " " << line.endPoints[i+1] << std::endl;
            octomap::point3d intersectedPoint;
            float distance;

            octomap::point3d startingPoint = line.endPoints[i].coord;
            octomap::point3d endPoint = line.endPoints[i+1].coord;
            float startingEnergy = line.endPoints[i].energy;

            /// Compute the point on the ray,
            /// where the point is the perpendicular foot of the particle to the ray.
            bool isDistanceComputed = computeDistancePointToLine(point, startingPoint, endPoint, distance, intersectedPoint);


            /// Compute the distance from the ray origin to the above point.
            float distanceOfRayToIntersectedPoint;
            if(i==0){
                distanceOfRayToIntersectedPoint = (intersectedPoint-startingPoint).norm();
            }else{
                distanceOfRayToIntersectedPoint = (intersectedPoint-startingPoint).norm() + fullDistance[i-1];                
            }

            float energyOfRay = 0;
            float particleWeight = 0;
            if(isDistanceComputed){

                /// Compute the propagation energy of the ray according to "The room acoustic rendering equation" Siltanen et al.
                energyOfRay = line.getAbsorptedEnergy((intersectedPoint-startingPoint).norm(), startingEnergy);

                /// Compute the weight of particles using the distance weight, which is the output of Gaussian function
                /// where the input is the shortest distance between the particle and the ray.
                particleWeight = computeWeightOfPaticle(distance, distanceOfRayToIntersectedPoint, energyOfRay, line.initialEnergy);
            }


            if( (maxWeightOfParticle < particleWeight) && isDistanceComputed)
            {
                maxWeightOfParticle = particleWeight;
                checkingIndex = i;

                fullDistance[i] = distanceOfRayToIntersectedPoint;

                rayDistance = fullDistance[i];
                intersectedPoint_ = intersectedPoint;
            }
            else
            {
                if(i == 0)
                    fullDistance[i] = (endPoint-startingPoint).norm();
                else
                    fullDistance[i] = (endPoint-startingPoint).norm() + fullDistance[i-1];
            }
        }


        return maxWeightOfParticle;
    }

    /// Compute the shortest distance from the particle to the ray.
    /// And 'intersectedPoint' is the perpendicular foot on the ray.
    bool ParticleFilter::computeDistancePointToLine(octomap::point3d point, octomap::point3d origin, octomap::point3d end, float& distance, octomap::point3d& intersectedPoint)
    {                              
        octomap::point3d v = end - origin;

        octomap::point3d w = point - origin;
        octomap::point3d z = point - end;

        /// Checking Orthogonal
        if(v.dot(w) < 0 || z.dot(v*(-1.0f)) < 0.0f ){         
            return false;
        }
        v.normalize();        

        float c1 = w.dot(v);
        float c2 = v.dot(v);

        if(c2 == 0)
            return false;

        float b = c1 / c2;

        intersectedPoint = origin + v*b;
        distance = (point-intersectedPoint).norm();
        return true;
    }

    /// Compute the likelihood of the particle using the distance weight explained in the 'computeDistancePointToLines' func.
    void ParticleFilter::computeImportanceSamplingWeight(std::vector<soundRaytracing::KeyRayEnergy>& rays)    // w_i
    {        
        resetWeightOfParticles();        
        //sumOfWeight = sumOfWeight * 0.5f;
        //sumOfWeight = sumOfWeight;

        if(rays.empty())
            return;

        int numberOfRays = rays.size();

        if(numberOfRays < 2){            
            isAcousticRay = false;
            return;
        }
        isAcousticRay = true;
        int intersectedCorrectly = false;        
        for(int i=0 ; i<numberOfRays ; i++)
        {
            /// for all particles
            for(int j=0 ; j<numberOfParticles ; j++)
            {                
                float rayDistance = 0;
                octomap::point3d intersectedPoint(0.0f, 0.0f, 0.0f);

                /// Compute the distance weight.
                float totalWeight = computeDistancePointToLines((particles.begin()+j)->position, rays[i], rayDistance, intersectedPoint, j);

                if(intersectedPoint.x() < 99999.9f && intersectedPoint.y() < 99999.9f && intersectedPoint.z() < 99999.9f){
                    intersectedCorrectly = true;
                }

                (particles.begin()+j)->addWeight(totalWeight);                
                sumOfWeight += totalWeight;
            }
        }

        if ( !intersectedCorrectly ){
            isAcousticRay = false;
        }

        std::sort(particles.begin(), particles.end());
    }



    /// sampling with sound model : p( x_t | x_(t-1) )
    void ParticleFilter::predictNextPositionOfParticle()  // p( x_t | x_(t-1) )
    {     
        if(isAcousticRay == false){
            resetParticles();
        }else{
            for(int i=0 ; i<numberOfParticles ; i++)
            {
                double r = getGaussianRandomValue(0.0, varOfSoundModel);
                double theta = getUniformRandomValue(0, 1) * PI;
                double phi = getUniformRandomValue(0, 1) * 2 * PI;
                (particles.begin()+i)->position.x() = (particles.begin()+i)->position.x() + r * std::sin(theta) * std::cos(phi);
                (particles.begin()+i)->position.y() = (particles.begin()+i)->position.y() + r * std::sin(theta) * std::sin(phi);
                (particles.begin()+i)->position.z() = (particles.begin()+i)->position.z() + r * std::cos(theta);
            }
        }
    }

    void ParticleFilter::resampleParticles()
    {
        int numberOfDeletedParticles = numberOfParticles - numberOfArrivalParticles;
        int indexDeletedParticles = 0;
        meanOfParticles.x() = 0; meanOfParticles.y() = 0; meanOfParticles.z() = 0;

        for(int i=numberOfDeletedParticles ; i<numberOfParticles ; i++)
        {

            int numberOfAddedParticles;
            if(i == (numberOfParticles-1))
                numberOfAddedParticles = numberOfParticles - indexDeletedParticles + 1;
            else
                numberOfAddedParticles = std::round(numberOfDeletedParticles*(particles[i].weight/sumOfWeight));

            meanOfParticles.x() += particles[i].position.x();
            meanOfParticles.y() += particles[i].position.y();
            meanOfParticles.z() += particles[i].position.z();

            for(int j=0 ; j<numberOfAddedParticles ; j++)
            {                
                particles[j+indexDeletedParticles].position = particles[i].position;
                meanOfParticles.x() += particles[i].position.x();
                meanOfParticles.y() += particles[i].position.y();
                meanOfParticles.z() += particles[i].position.z();
            }
            indexDeletedParticles += numberOfAddedParticles;
        }
        meanOfParticles.x() = meanOfParticles.x()/(float)numberOfParticles;
        meanOfParticles.y() = meanOfParticles.y()/(float)numberOfParticles;
        meanOfParticles.z() = meanOfParticles.z()/(float)numberOfParticles;        
    }

    /// This function is presented for visualizing the particles on Rviz.
    void ParticleFilter::transferParticleToPointCloud(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &pointClouds)
    {
        varianceOfParticles.x() = 0; varianceOfParticles.y() = 0; varianceOfParticles.z() = 0;
        double xx=0, xy=0, xz=0, yx=0, yy=0, yz=0, zx=0, zy=0, zz=0;
        for(int i=0 ; i<numberOfParticles ; i++)
        {
            pointClouds.push_back(pcl::PointXYZ(particles[i].position.x(), particles[i].position.y(), particles[i].position.z()));
            varianceOfParticles.x() += (particles[i].position.x()-meanOfParticles.x()) * (particles[i].position.x()-meanOfParticles.x());
            varianceOfParticles.y() += (particles[i].position.y()-meanOfParticles.y()) * (particles[i].position.y()-meanOfParticles.y());
            varianceOfParticles.z() += (particles[i].position.z()-meanOfParticles.z()) * (particles[i].position.z()-meanOfParticles.z());
            xx += (particles[i].position.x()-meanOfParticles.x()) * (particles[i].position.x()-meanOfParticles.x());
            xy += (particles[i].position.x()-meanOfParticles.x()) * (particles[i].position.y()-meanOfParticles.y());
            xz += (particles[i].position.x()-meanOfParticles.x()) * (particles[i].position.z()-meanOfParticles.z());

            yx += (particles[i].position.y()-meanOfParticles.y()) * (particles[i].position.x()-meanOfParticles.x());
            yy += (particles[i].position.y()-meanOfParticles.y()) * (particles[i].position.y()-meanOfParticles.y());
            yz += (particles[i].position.y()-meanOfParticles.y()) * (particles[i].position.z()-meanOfParticles.z());

            zx += (particles[i].position.z()-meanOfParticles.z()) * (particles[i].position.x()-meanOfParticles.x());
            zy += (particles[i].position.z()-meanOfParticles.z()) * (particles[i].position.y()-meanOfParticles.y());
            zz += (particles[i].position.z()-meanOfParticles.z()) * (particles[i].position.z()-meanOfParticles.z());         
        }
        varianceOfParticles.x() = varianceOfParticles.x()/(float)numberOfParticles;
        varianceOfParticles.y() = varianceOfParticles.y()/(float)numberOfParticles;
        varianceOfParticles.z() = varianceOfParticles.z()/(float)numberOfParticles;

        xx = xx/(float)numberOfParticles;
        xy = xy/(float)numberOfParticles;
        xz = xz/(float)numberOfParticles;

        yx = yx/(float)numberOfParticles;
        yy = yy/(float)numberOfParticles;
        yz = yz/(float)numberOfParticles;

        zx = zx/(float)numberOfParticles;
        zy = zy/(float)numberOfParticles;
        zz = zz/(float)numberOfParticles;

        covarianceOfParticles.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
    }

    bool ParticleFilter::getSoundSourcePosition(octomap::point3d &goalPosition, double& determinantCov, tf::Matrix3x3& covM)
    {
        /// This (determinantCov) is the generalized variance, which is corresponding the convergence of particles.
        determinantCov = covarianceOfParticles.determinant();
        goalPosition = meanOfParticles;
        covM = covarianceOfParticles;
        //std::cout << "  !! determinant: " << determinantCov << std::endl;
        if(determinantCov < 0.06){
            isSoundDetected = true;
            return true;
        }
        else{
            return false;
        }
    }
}
