
#undef max
#undef min
#include <limits>

#ifdef _OPENMP
  #include <omp.h>
#endif

namespace octomap {
    bool KeyRayEnergyTracing::computeRayKeys(octomap::ColorOcTree* octomap, const point3d& origin, const point3d& end, KeyRayEnergy& ray, float energy, unsigned& distance) const
    {
        // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
        // basically: DDA in 3D

        ray.reset();

        OcTreeKey key_origin, key_end;
        if ( !OcTreeBaseImpl<NODE,I>::coordToKeyChecked(origin, key_origin) ||
             !OcTreeBaseImpl<NODE,I>::coordToKeyChecked(end, key_end) ) {
          OCTOMAP_WARNING_STR("coordinates ( "
                    << origin << " -> " << end << ") out of bounds in computeRayKeys");
          return false;
        }


        if (key_origin == key_end)
          return true; // same tree cell, we're done.

        ray.addKey(key_origin);

        // Initialization phase -------------------------------------------------------

        point3d direction = (end - origin);
        float length = (float) direction.norm();
        direction /= length; // normalize vector

        int    step[3];
        double tMax[3];
        double tDelta[3];

        OcTreeKey current_key = key_origin;

        for(unsigned int i=0; i < 3; ++i) {
          // compute step direction
          if (direction(i) > 0.0) step[i] =  1;
          else if (direction(i) < 0.0)   step[i] = -1;
          else step[i] = 0;

          // compute tMax, tDelta
          if (step[i] != 0) {
            // corner point of voxel (in direction of ray)
            double voxelBorder = this->keyToCoord(current_key[i]);
            voxelBorder += (float) (step[i] * this->resolution * 0.5);

            tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
            tDelta[i] = this->resolution / fabs( direction(i) );
          }
          else {
            tMax[i] =  std::numeric_limits<double>::max( );
            tDelta[i] = std::numeric_limits<double>::max( );
          }
        }

        // Incremental phase  ---------------------------------------------------------

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

          // advance in direction "dim"
          current_key[dim] += step[dim];
          tMax[dim] += tDelta[dim];

          assert (current_key[dim] < 2*this->tree_max_val);

          // reached endpoint, key equv?
          if (current_key == key_end) {
            done = true;
            break;
          }
          else {

            // reached endpoint world coords?
            // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
            double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);
            // if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
            // However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel key_end
            if (dist_from_origin > length) {
              done = true;
              break;
            }

            else {  // continue to add freespace cells
              ray.addKey(current_key);
            }
          }

          assert ( ray.size() < ray.sizeMax() - 1);

        } // end while

        return true;
    }
}
