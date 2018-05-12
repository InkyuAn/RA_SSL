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
        return getGaussianDistribution(distance);

    }

    float ParticleFilter::computeDistancePointToLines(octomap::point3d point, soundRaytracing::KeyRayEnergy& line, float &rayDistance, octomap::point3d& intersectedPoint_, int j)  // with reflection
    {        
        float maxWeightOfParticle = 0.0f;
        //float fullDistance[line.endPoints.size()-1] = {0,};
        std::vector<float> fullDistance;
        fullDistance.resize(line.endPoints.size()-1, 0.0f);
        int checkingIndex = 0;

        rayDistance = 999999.9f;
        intersectedPoint_ = octomap::point3d(999999.9f, 999999.9f, 999999.9f);

        // Projected into XY plane
        //octomap::point3d originPostionOfRay = octomap::point3d(line.endPoints[0].coord.x(), line.endPoints[0].coord.y(), 0.0f);
        octomap::point3d originPostionOfRay = octomap::point3d(robotOrigin.x(), robotOrigin.y(), 0.0f);

        if(j == 0)
            std::cout << "compute distance point to line: " << line.initialEnergy << " " << line.endPoints.size() << " " << line.ray.size() << std::endl;
        for(int i = 0 ; i<line.endPoints.size()-1 ; i++)
        {
            //std::cout << "endPoint: " << line.endPoints[i] << " " << line.endPoints[i+1] << std::endl;
            octomap::point3d intersectedPoint;
            float distance;

            octomap::point3d startingPoint = line.endPoints[i].coord;
            octomap::point3d endPoint = line.endPoints[i+1].coord;
            float startingEnergy = line.endPoints[i].energy;

            /*auto printpoint = [](const char *name, octomap::point3d &x){
                std::cout << name << ": " << x.x() << " " << x.y() << " " << x.z() << std::endl;
            };*/

            //printpoint("point", point);
            //printpoint("startingPoint", startingPoint);
            //printpoint("endPoint", endPoint);

            //float distance = computeDistancePointToLine(point, startingPoint, endPoint, intersectedPoint);
            bool isDistanceComputed = computeDistancePointToLine(point, startingPoint, endPoint, distance, intersectedPoint);

            // Projected into XY plane
            octomap::point3d tempIntersected = octomap::point3d(intersectedPoint.x(), intersectedPoint.y(), 0.0f);
            //std::cout << "&& intersected point: " << tempIntersected.x() << " " << tempIntersected.y() << std::endl;
            //(tempIntersected-originPostionOfRay).norm()

            if(tempIntersected.y() > 0.5f){
                if((tempIntersected).norm() < 1.5f)
                    continue;
            }
            else if(tempIntersected.y() < -0.5f){
                if((tempIntersected).norm() < 0.6f)
                    continue;
            }
            else{
                if((tempIntersected).norm() < 1.0f)
                    continue;
            }



            float distanceOfRayToIntersectedPoint;

            if(i==0){
                distanceOfRayToIntersectedPoint = (intersectedPoint-startingPoint).norm();
            }else{
                distanceOfRayToIntersectedPoint = (intersectedPoint-startingPoint).norm() + fullDistance[i-1];
                //std::cout << "fullDistance[i-1]" << fullDistance[i-1] << std::endl;
            }

            float energyOfRay = 0;
            float particleWeight = 0;
            if(isDistanceComputed){
                //for(int rayIdx = 0 ; rayIdx < line.ray.size() ; rayIdx++){
                   //octomap::point3d tempRayNode = Octree->keyToCoord(octomap::OcTreeKey(line.ray[rayIdx].k[0], line.ray[rayIdx].k[1], line.ray[rayIdx].k[2]));

                   energyOfRay = line.getAbsorptedEnergy((intersectedPoint-startingPoint).norm(), startingEnergy);


                if(j == 0)
                    std::cout << " [" << i << "]" << energyOfRay;
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
        if(j == 0)
            std::cout << std::endl;

        return maxWeightOfParticle;
    }

    bool ParticleFilter::computeDistancePointToLine(octomap::point3d point, octomap::point3d origin, octomap::point3d end, float& distance, octomap::point3d& intersectedPoint)
    {                       
        /*auto printpoint = [](const char *name, octomap::point3d &x){
            std::cout << name << ": " << x.x() << " " << x.y() << " " << x.z() << std::endl;
        };*/

        octomap::point3d v = end - origin;

        octomap::point3d w = point - origin;
        octomap::point3d z = point - end;

        //printpoint("v", v);
        //printpoint("w", w);
        //printpoint("z", z);

        // Checking Orthogonal
        if(v.dot(w) < 0 || z.dot(v*(-1.0f)) < 0.0f ){
            //intersectedPoint = octomap::point3d(0.0, 0.0, 0.0);
            return false;
        }
        v.normalize();
        //std::cout << "After retur, " << std::endl;




        float c1 = w.dot(v);
        float c2 = v.dot(v);

        if(c2 == 0)
            return false;

        float b = c1 / c2;

        //octomap::point3d Pb = origin + v*b;
        intersectedPoint = origin + v*b;
        distance = (point-intersectedPoint).norm();
        return true;
    }

    void ParticleFilter::computeImportanceSamplingWeight(std::vector<soundRaytracing::KeyRayEnergy>& rays)    // w_i
    {
        //int raySize = rays.size();
        resetWeightOfParticles();
        float preSumOfWeight = sumOfWeight;
        //sumOfWeight = sumOfWeight * 0.5f;
        sumOfWeight = sumOfWeight;

        if(rays.empty())
            return;

        int numberOfRays = rays.size();

        if(numberOfRays < 2){
            //std::cout << "numberOfRays == 0" << std::endl;
            isAcousticRay = false;
            return;
        }
        isAcousticRay = true;
        int intersectedCorrectly = false;
        // Back up 2017 06 26
        for(int i=0 ; i<numberOfRays ; i++)
        {
            //soundRaytracing::KeyRayEnergy ray_ = rays[i];

            /*std::cout << "  endPoints: ";
            for(int k=0 ; k<rays[i].endPoints.size()-1 ; k++){
                std::cout << rays[i].endPoints[k] << " ";
            }
            std::cout << std::endl;*/

            // for all particles
            for(int j=0 ; j<numberOfParticles ; j++)
            {
                //ps_soundPotentialField.PotentialRays[i].ray[0].energy;
                float rayDistance = 0;
                octomap::point3d intersectedPoint(0.0f, 0.0f, 0.0f);


                //float distance = computeDistancePointToLines((particles.begin()+j)->position, rays[i], rayDistance, intersectedPoint);
                //float totalWeight = computeDistancePointToLines((particles.begin()+j)->position, rays[i], rayDistance, intersectedPoint);
                float totalWeight = computeDistancePointToLines((particles.begin()+j)->position, rays[i], rayDistance, intersectedPoint, j);

                //intersectedPoint_ = octomap::point3d(999999.9f, 999999.9f, 999999.9f);
                if(intersectedPoint.x() < 99999.9f && intersectedPoint.y() < 99999.9f && intersectedPoint.z() < 99999.9f){
                    intersectedCorrectly = true;
                }

                (particles.begin()+j)->addWeight(totalWeight);
                //sumOfWeight += weight;
                sumOfWeight += totalWeight;
            }
            //std::cout << std::endl;
        }

        if ( !intersectedCorrectly ){
            isAcousticRay = false;
        }

        std::sort(particles.begin(), particles.end());
    }



    // sampling with sound model : p( x_t | x_(t-1) )
    void ParticleFilter::predictNextPositionOfParticle()  // p( x_t | x_(t-1) )
    {
        //resetParticles
        if(isAcousticRay == false){// && covarianceOfParticles.determinant() > 1.0){
            //isSoundDetected = false;
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

        //printParticlePosition();
    }

    void ParticleFilter::resampleParticles()
    {
        int numberOfDeletedParticles = numberOfParticles - numberOfArrivalParticles;
        int indexDeletedParticles = 0;
        meanOfParticles.x() = 0; meanOfParticles.y() = 0; meanOfParticles.z() = 0;

        //std::cout << ">>>>>>numberOfDeletedParticles:  " << numberOfDeletedParticles << std::endl;
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
            //std::cout << ">>>>>>numberOfAddedParticles: " << numberOfAddedParticles << std::endl;
            //std::cout << ">>>>>>indexDeletedParticles: " << indexDeletedParticles << std::endl;

            for(int j=0 ; j<numberOfAddedParticles ; j++)
            {
                //std::cout << "  >>>>>> index: " << j+indexDeletedParticles << std::endl;
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
        //std::cout << "  !!meanOfParticle: " << meanOfParticles.x() << " " << meanOfParticles.y() << " " << meanOfParticles.z() << std::endl;
    }

    void ParticleFilter::transferParticleToPointCloud(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> &pointClouds)
    {
        //pointClouds.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));

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
            //covarianceOfParticles.setValue();

            //std::cout << "*";
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

        //std::cout << std::endl;
        //std::cout << "  !!varianceOfParticles: " << varianceOfParticles.x() << " " << varianceOfParticles.y() << " " << varianceOfParticles.z() << std::endl;
    }

    bool ParticleFilter::getSoundSourcePosition(octomap::point3d &goalPosition, double& determinantCov, tf::Matrix3x3& covM)
    {

        //double determinantCov = covarianceOfParticles.determinant();
        determinantCov = covarianceOfParticles.determinant();
        goalPosition = meanOfParticles;
        covM = covarianceOfParticles;
        //std::cout << "  !! determinant: " << covarianceOfParticles.determinant() << std::endl;
        /*if(varianceOfParticles.x() < 10.0f &&
                varianceOfParticles.y() < 10.0f &&
                varianceOfParticles.z() < 10.0f)*/
        //if(determinantCov < 0.05)
        if(determinantCov < 0.06){
            isSoundDetected = true;
            return true;
        }
        else{
            return false;
        }
    }
}
