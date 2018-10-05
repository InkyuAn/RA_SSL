
#include <acousticRaytracing.h>

#undef max
#undef min
#include <limits>

#ifdef _OPENMP
  #include <omp.h>
#endif

#define TEST_COMPUTE_NORMAL 1
namespace soundRaytracing {

    /// Compute the energy absortion of acoustic rays
    float KeyRayEnergy::getAbsorptedEnergy(float distance)
    {        
        return (initialEnergy*std::exp(distance * alpha));
    }
    float KeyRayEnergy::getAbsorptedEnergy(float distance, float staritingEnergy)
    {        
        return (staritingEnergy*std::exp(distance * alpha));
    }

    /// Compute the normal on Octomap using SVD
    bool KeyRayEnergy::computeNormalVector(const octomap::ColorOcTree* octree, const octomap::OcTreeKey& keyValue, octomap::point3d& normals)
    {
        /// the number of a local cube
        int numOfCube = 5;

        int middleOfCube = std::floor(numOfCube/2);

        /// intialize Matrix A
        double A[125][3] = {0, };

        int maxTreeDepth = octree->getTreeDepth();

        int numOfOccupiedCell = 0;
        double middleOfOccupiedCell[3] = {0.0, 0.0, 0.0};

        /// Extract the local cube
        for(int i=0 ; i<numOfCube ; i++){
            for(int j=0 ; j<numOfCube ; j++){
                for(int k=0 ; k<numOfCube ; k++){

                    octomap::OcTreeKey tempKey(keyValue[0] - middleOfCube + i, keyValue[1] - middleOfCube + j, keyValue[2] - middleOfCube + k);
                    octomap::OcTreeNode* tempNode = octree->search(tempKey, maxTreeDepth);
                    if(tempNode != NULL){
                        if(octree->isNodeOccupied(tempNode)){                            
                            middleOfOccupiedCell[0] += (double)i;
                            middleOfOccupiedCell[1] += (double)j;
                            middleOfOccupiedCell[2] += (double)k;
                            A[numOfOccupiedCell][0] = (double)i;
                            A[numOfOccupiedCell][1] = (double)j;
                            A[numOfOccupiedCell][2] = (double)k;
                            numOfOccupiedCell++;
                        }

                    }else{
                    }

                }
            }
        }

        middleOfOccupiedCell[0] /= (double)numOfOccupiedCell;
        middleOfOccupiedCell[1] /= (double)numOfOccupiedCell;
        middleOfOccupiedCell[2] /= (double)numOfOccupiedCell;

        for(int i=0 ; i<numOfOccupiedCell ; i++){
            A[i][0] -= middleOfOccupiedCell[0];
            A[i][1] -= middleOfOccupiedCell[1];
            A[i][2] -= middleOfOccupiedCell[2];
        }


        /// Apply SVD to extract the normal
        double U[125][3];
        double V[3][3];
        double singular_values[3];
        double dummy_array[3];

        int err = Singular_Value_Decomposition((double*)A, numOfOccupiedCell, 3, (double*)U, singular_values, (double*)V, (double*) dummy_array);

        if(err < 0){
            return false;
        } else{
            octomap::point3d normal;
            normal.x() = V[2][0];
            normal.y() = V[2][1];
            normal.z() = V[2][2];            
            normals = normal;
            return true;
        }
    }

    /// Acoustic ray tracing
    void KeyRayEnergy::computeEnergyRayKeys(octomap::ColorOcTree* octree, const octomap::point3d& originP, const octomap::point3d& directionP, double maxRange)
    {
        unsigned reflectionIdx = 0; // reflection index
        double maxRayDistance = maxRange;    // 10m

        /// initialize ray
        octomap::point3d direction = directionP;
        octomap::point3d origin = originP + directionP*0.5;

        /// insert the starting point
        endPoint_ tempOrigin;
        tempOrigin.coord = origin;
        tempOrigin.energy = initialEnergy;
        endPoints.push_back(tempOrigin);


        float originEnergy = initialEnergy;
        float endPointEnergy = 0.0f;

        /// Count reflection is less than 5
        while(reflectionIdx < 5) {

            /// Propagate acoustic ray
            bool checkMaxDistance = this->traceEnergyRayKeys(octree, origin, direction, originEnergy, endPointEnergy, maxRayDistance, reflectionIdx);

            originEnergy = endPointEnergy;

            maxRayDistance -= this->distance;

            if(!checkMaxDistance)   // if ray distance > 10m, break
                break;

            /// Compute normal vector
            octomap::point3d normals(0,0,0);   // normal vector
            octomap::OcTreeKey endNodeKey((this->end()-1)->k[0], (this->end()-1)->k[1], (this->end()-1)->k[2]); // get end node key of ray
            octomap::point3d endNode = octree->keyToCoord(endNodeKey);

            bool gotNormals = false;    // flag of normal

            if((this->end()-1)->k[2] > ceilingHeight-2) { // Ceiling                
                normals = octomap::point3d(0.0, 0.0, -1.0);
                /// Absorption coefficient is set by 1.1
                originEnergy = originEnergy * 1.1;
                gotNormals = true;
            }            
            else if((this->end()-1)->k[2] < bottomHeight+2) {  // Bottom                
                normals = octomap::point3d(0.0, 0.0, 1.0);
                /// Absorption coefficient is set by 1.1
                originEnergy = originEnergy * 1.1;
                gotNormals = true;
            }
            else {  // other cases
                /// Absorption coefficient is set by 1.1
                originEnergy = originEnergy * 1.1;
                gotNormals = computeNormalVector(octree, endNodeKey, normals);                
            }

            if(gotNormals)
            {                
                octomap::point3d normalSum(0, 0, 0);

                octomap::point3d normal = normals;

                if(normal.dot(direction) < 0){
                    normalSum = normal;
                }else{
                    normal *= -1.0;
                    if(normal.dot(direction) < 0)
                        normalSum = normal;
                }

                if(normal.norm() > 0)
                {                    
                    endNodeKey = octomap::OcTreeKey((this->end()-1)->k[0], (this->end()-1)->k[1], (this->end()-1)->k[2]); // get end node key of ray
                    octomap::point3d endNodeCoord(octree->keyToCoord(endNodeKey)); // get end node coordinate of ray
                    /// insert the end point
                    endPoint_ tempNode;
                    tempNode.coord = endNodeCoord;
                    tempNode.energy = (this->end()-1)->energy;
                    endPoints.push_back(tempNode);

                    /// Apply a reflection
                    octomap::point3d incidentDir = direction * -1;
                    octomap::point3d outgoingDir = normal*(2.0*incidentDir.dot(normal)) - incidentDir;   // calculate outgoing ray vector
                    // Set parameters of next ray
                    direction = outgoingDir;
                    origin = endNodeCoord;                    
                }
                else{
                    octomap::point3d endNodeCoord(octree->keyToCoord(endNodeKey)); // get end node coordinate of ray
                    // insert the end point
                    endPoint_ tempNode;
                    tempNode.coord = endNodeCoord;
                    tempNode.energy = (this->end()-1)->energy;
                    endPoints.push_back(tempNode);
                    //endPoints.push_back(endNodeCoord);
                    break;  // Break point (Couldn't find normal using dot product)
                }

            }
            else{
                octomap::point3d endNodeCoord(octree->keyToCoord(endNodeKey)); // get end node coordinate of ray
                // insert the end point
                endPoint_ tempNode;
                tempNode.coord = endNodeCoord;
                tempNode.energy = (this->end()-1)->energy;
                endPoints.push_back(tempNode);
                break;  // Break point (Couldn't find normal using Marching cube)
            }
            reflectionIdx++;
        }

    }

    bool KeyRayEnergy::traceEnergyRayKeys(octomap::ColorOcTree* octree, const octomap::point3d& origin, const octomap::point3d& directionP,
                                          float originE, float &endPointE , double maxRange, int rIndex)
    {
        // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
        // basically: DDA in 3D

        octomap::OcTreeKey key_origin;        

        if ( !octree->coordToKeyChecked(origin, key_origin)) {
            OCTOMAP_WARNING_STR("coordinates ( "
                    << origin << ") out of bounds in computeRayKeys");
            return false;
        }

        octomap::ColorOcTreeNode* startingNode = octree->search(key_origin);
        if (startingNode)
        {
            if(octree->isNodeOccupied(startingNode))
            {                
                return true;
            }
        }

        OcTreeKeyEnergy keyEnergy_origin(key_origin, originE, rIndex);
        this->addKey(keyEnergy_origin);


        // Initialization phase -------------------------------------------------------

        octomap::point3d direction = directionP.normalized();

        int    step[3];
        double tMax[3];
        double tDelta[3];

        octomap::OcTreeKey current_key = key_origin;

        for(unsigned int i=0; i < 3; ++i) {
            // compute step direction
            if (direction(i) > 0.0) step[i] =  1;
            else if (direction(i) < 0.0)   step[i] = -1;
            else step[i] = 0;

            // compute tMax, tDelta
            if (step[i] != 0) {
                // corner point of voxel (in direction of ray)
                double voxelBorder = octree->keyToCoord(current_key[i]);
                voxelBorder += (float) (step[i] * octree->getResolution() * 0.5);

                tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
                tDelta[i] = octree->getResolution() / fabs( direction(i) );
            }
            else {
                tMax[i] =  std::numeric_limits<double>::max( );
                tDelta[i] = std::numeric_limits<double>::max( );
            }
        }

        if (step[0] == 0 && step[1] == 0 && step[2] == 0){
            OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
            return false;
        }

        // for speedup:
        double maxrange_sq = maxRange *maxRange;
        //double maxrange_sq = maxRange;

        // Incremental phase  ---------------------------------------------------------

        float energy = 0.0f;
        float preEnergy = -1.0f;
        float diffEnergy = 0.0f;

        bool done = false;
        while (!done) {

            unsigned int dim;

            // find minimum tMax:
            if (tMax[0] < tMax[1]){
                if (tMax[0] < tMax[2]) dim = 0;
                else                   dim = 2;
            }
            else {
                if (tMax[1] < tMax[2]) dim = 1;
                else                   dim = 2;
            }

            // check for overflow:
            if ((step[dim] < 0 && current_key[dim] == 0)
                  || (step[dim] > 0 && current_key[dim] == 2* (32767))) //octomap->tree_max_val - 1
            {
                OCTOMAP_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
                // return border point nevertheless:
                return false;
            }

            // advance in direction "dim"
            current_key[dim] += step[dim];
            tMax[dim] += tDelta[dim];

            // reached endpoint, key equv?
            // 20170228 IK

            octomap::ColorOcTreeNode* currentNode = octree->search(current_key);
            if(currentNode)
            {
                if(octree->isNodeOccupied(currentNode))
                {
                    done = true;
                    //std::cout << "break raycast 1" << std::endl;
                    break;
                }
            }


            double dist_from_origin_sq(0.0);
            for(unsigned int j=0 ; j<3 ; j++)
            {
                octomap::point3d current_coordinate = octree->keyToCoord(current_key);
                dist_from_origin_sq += ((current_coordinate(j) - origin(j)) * (current_coordinate(j) - origin(j)));
            }
            distance = std::sqrt(dist_from_origin_sq);

            //std::cout << "distance: " << distance << std::endl;

            //float energy = initialEnergy*std::exp(distance * alpha);
            energy = originE*std::exp(distance * alpha);
            if(preEnergy < 0){
                diffEnergy = 0.0f;
            }else{
                diffEnergy = energy - preEnergy;
            }
            preEnergy = energy;
            // ceiling & bottom & distance            
            if (dist_from_origin_sq > maxrange_sq || energy >= 2) {            
                done = true;
                //std::cout << "break raycast 2" << std::endl;
                return false;
                break;
            }else if (current_key.k[2] >= ceilingHeight || current_key.k[2] <= bottomHeight) {
                done = true;
                //std::cout << "break raycast 2" << std::endl;
                break;
            }
            else {
                //std::cout << "  current_key: " << current_key.k[0]  << " " << current_key.k[1] << " " << current_key.k[2] << std::endl;
                OcTreeKeyEnergy keyEnergy_current(current_key, energy, rIndex);
                this->addKey(keyEnergy_current);
            }

            assert ( this->size() < this->sizeMax() - 1);

        } // end while
        endPointE = energy;
        return true;
    }

}
