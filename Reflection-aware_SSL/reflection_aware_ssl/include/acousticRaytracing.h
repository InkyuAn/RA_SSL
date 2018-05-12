/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RAYTRACING_H
#define RAYTRACING_H

/* According to c++ standard including this header has no practical effect
 * but it can be used to determine the c++ standard library implementation.
 */
#include <ciso646>

#include <assert.h>

#include <octomap/ColorOcTree.h>

#include <rayBasedSSL_typedef.h>

//#include <nav_msgs/OccupancyGrid.h>

#include "SVD/singular_value_decomposition.h"


#include <ros/ros.h>


/// It's based on Octomap source code
/// I make it easy to use in my project.
///
namespace soundRaytracing {

    /**
    * OcTreeKeyEnergy is a container class for internal key addressing. The keys count the
    * number of cells (voxels) from the origin as discrete address of a voxel.
    * @see OcTreeBaseImpl::coordToKey() and OcTreeBaseImpl::keyToCoord() for conversions.
    */
    class OcTreeKeyEnergy {

    public:
    OcTreeKeyEnergy () {}
    OcTreeKeyEnergy (unsigned short int a, unsigned short int b, unsigned short int c, float en, int rIndex)
      { k[0] = a; k[1] = b; k[2] = c; energy = en; reflectIndex = rIndex;}
    OcTreeKeyEnergy(const OcTreeKeyEnergy& other){
      k[0] = other.k[0]; k[1] = other.k[1]; k[2] = other.k[2]; energy = other.energy; reflectIndex=other.reflectIndex;
    }
    OcTreeKeyEnergy(const octomap::OcTreeKey& other, float en, int rIndex){
      k[0] = other.k[0]; k[1] = other.k[1]; k[2] = other.k[2]; energy = en; reflectIndex = rIndex;
    }
    bool operator== (const OcTreeKeyEnergy &other) const {
      return ((k[0] == other[0]) && (k[1] == other[1]) && (k[2] == other[2]));
    }
    bool operator!= (const OcTreeKeyEnergy &other) const {
      return( (k[0] != other[0]) || (k[1] != other[1]) || (k[2] != other[2]) );
    }
    void setEnergy(float en)
    { energy = en; }
    float getEnergy()
    { return energy; }
    OcTreeKeyEnergy& operator=(const OcTreeKeyEnergy& other){
      k[0] = other.k[0]; k[1] = other.k[1]; k[2] = other.k[2]; energy = other.energy; reflectIndex=other.reflectIndex;
      return *this;
    }
    const unsigned short int& operator[] (unsigned int i) const {
      return k[i];
    }
    unsigned short int& operator[] (unsigned int i) {
      return k[i];
    }

    int reflectIndex;
    unsigned short int k[3];
    float energy;

    /// Provides a hash function on Keys
    struct KeyHash{
      size_t operator()(const OcTreeKeyEnergy& key) const{
        // a hashing function
        return key.k[0] + 1337*key.k[1] + 345637*key.k[2];
      }
    };

    };
  
  /**
   * Data structure to efficiently compute the nodes to update from a scan
   * insertion using a hash set.
   * @note you need to use boost::unordered_set instead if your compiler does not
   * yet support tr1!
   */
  //typedef unordered_ns::unordered_set<OcTreeKeyEnergy, OcTreeKeyEnergy::KeyHash> KeySet;

  /**
   * Data structrure to efficiently track changed nodes as a combination of
   * OcTreeKeyEnergys and a bool flag (to denote newly created nodes)
   *
   */
  //typedef unordered_ns::unordered_map<OcTreeKeyEnergy, bool, OcTreeKeyEnergy::KeyHash> KeyBoolMap;
    class KeyRayEnergy {
    public:

    
        KeyRayEnergy (float en, unsigned short int ceiling, unsigned short int bottom, int index)
            : distance(0),
              alpha(0),
              initialEnergy(en),
              ceilingHeight(ceiling), bottomHeight(bottom),
              rayIndex(index)
        {
            //initialEnergy = en;
            //ceilingHeight = ceiling; bottomHeight = bottom;
            //mapInformation_octree = NULL;

            //rayIndex = index;
            endPoints.clear();
            ray.resize(100000);
            reset();

        }KeyRayEnergy (float en, unsigned short int ceiling, unsigned short int bottom)
            : distance(0), numOfray(0),
              alpha(0),
              initialEnergy(en),
              ceilingHeight(ceiling), bottomHeight(bottom),
              rayIndex(-1)
        {

            endPoints.clear();
            //rayIndex = -1;
            //distance = 0;
            ray.resize(100000);
            reset();
        }
        void reset() {            
          end_of_ray = begin();
          numOfray = 0;
          endPoints.clear();
        }
        void addKey(OcTreeKeyEnergy& k) {
          assert(end_of_ray != ray.end());
          *end_of_ray = k;
          //*end_of_ray
          //std::cout << (*end_of_ray).energy << std::endl;
          numOfray++;
          end_of_ray++;
        }

        void init(raybased_soundlocalization::soundAttenuation atmosphericAbsorption, float freq){
            float freqSquaredTwo = freq * freq;
            frequency = freq;
            alpha = float(
                        freqSquaredTwo * (atmosphericAbsorption.getA()
                                      + atmosphericAbsorption.getB()/(atmosphericAbsorption.getFro()+(freqSquaredTwo/atmosphericAbsorption.getFro()))
                                      + atmosphericAbsorption.getC()/(atmosphericAbsorption.getFrn()+(freqSquaredTwo/atmosphericAbsorption.getFrn())))
                          );
            //std::cout << "alpha: " << alpha << std::endl;
        }

        unsigned int size() const { return numOfray; }
        unsigned int sizeMax() const { return 100000; }

        typedef std::vector<OcTreeKeyEnergy>::iterator iterator;
        typedef std::vector<OcTreeKeyEnergy>::const_iterator const_iterator;
        typedef std::vector<OcTreeKeyEnergy>::reverse_iterator reverse_iterator;

        iterator begin() { return ray.begin(); }
        iterator end() { return end_of_ray; }
        const_iterator begin() const { return ray.begin(); }
        const_iterator end() const   { return end_of_ray; }

        reverse_iterator rbegin() { return (reverse_iterator) end_of_ray; }
        reverse_iterator rend() { return ray.rend(); }

        //octomap::ColorOcTree* mapInformation_octree;

    public:

        std::vector<OcTreeKeyEnergy> ray;
        std::vector<OcTreeKeyEnergy>::iterator end_of_ray;

        //std::vector<octomap::point3d> endPoints;

        struct endPoint_{
            octomap::point3d coord;
            float energy;
        };

        std::vector<endPoint_> endPoints;

        float initialEnergy;
        float distance;
        unsigned short int ceilingHeight;
        unsigned short int bottomHeight;
        int rayIndex;
        int numOfray;

        float alpha;
        float frequency;

        float getEnergy(void) const { return initialEnergy; }

        //float getEnergy(octomap::point3d intersectedPoint);
        float getAbsorptedEnergy(float distance);
        float getAbsorptedEnergy(float distance, float staritingEnergy);


        //bool computeNormalVector(const octomap::ColorOcTree* octree, const octomap::OcTreeKey& keyValue, octomap::point3d& normals);
        bool computeNormalVector(const octomap::ColorOcTree* octree, const octomap::OcTreeKey& keyValue, octomap::point3d& normals);

        void computeEnergyRayKeys(octomap::ColorOcTree* octomap, const octomap::point3d& originP, const octomap:: point3d& directionP, double maxRange);

        //bool traceEnergyRayKeys(octomap::ColorOcTree* octomap, const octomap::point3d& origin, const octomap:: point3d& directionP, double maxRange);
        bool traceEnergyRayKeys(octomap::ColorOcTree* octree, const octomap::point3d& origin, const octomap::point3d& directionP, float originE, float &endPointE , double maxRange, int reflectionIdx);

    };

} // namespace



//#include <raytracing.hxx>

#endif

