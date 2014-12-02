/*
 * Copyright (c) 2013-2014, Dario Lodi Rizzini, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RAY_TRACER_HPP
#define RAY_TRACER_HPP

#include <set>

template <typename PointT>
SignedIntensityOccupancyPolicy::OccupancyStatus SignedIntensityOccupancyPolicy::getOccupancyStatus(const PointT& p)
{
  float intensity;
  bool exists;
  // It checks the value of point field "intensity" if it exists
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<PointT, float> (p, "intensity", exists, intensity));
  if (!exists) {
    std::cerr << __FILE__ << "," << __LINE__ << ": field \"intensity\" does not exists for this type of points" << std::endl;
    return UNKNOWN;
  }
  // A convention on intensity to check whether 
  if (std::abs(intensity) < EPS) return UNKNOWN;
  else if (intensity > 0.0f) return OCCUPIED;
  else return FREE;
}

template <typename PointT>
ProbabilisticOccupancyPolicy::OccupancyStatus ProbabilisticOccupancyPolicy::getOccupancyStatus(const PointT& p)
{
  float intensity;
  bool exists;
  // It checks the value of point field "intensity" if it exists
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<PointT, float> (p, "intensity", exists, intensity));
  if (!exists) {
    std::cerr << __FILE__ << "," << __LINE__ << ": field \"intensity\" does not exists for this type of points" << std::endl;
    return UNKNOWN;
  }
  // A convention on intensity to check whether 
  if (std::abs(intensity) < TOLLERANCE) return FREE;
  else if (intensity > 1.0 - TOLLERANCE) return OCCUPIED;
  else return UNKNOWN;
}

// --------------------------------------------------------
// RAY TRACING
// --------------------------------------------------------

//// More efficient implementation: it sorts all the boxes w.r.t. the distance to the ray origin  
//// and stop box intersection check at the FIRST intersection which is also the closest one
//// thanks to the order!
//template <typename PointT>
//void PinholeRayTracer::evaluate(const pcl::PointCloud<PointT>& cloud,pcl::PointIndices& indices)
//{
//  Eigen::Vector3f rayo;                // ray origin
//  std::vector<Eigen::Vector3f> rayds;  // ray directions (variable)
//  Eigen::Vector3f bmin;  // box min coordinates
//  Eigen::Vector3f bmax;  // box max coordinates
//  Eigen::Vector3f bctr;  // box center coordinates
//  Eigen::Vector3f rayi;

//  // Sorts a copy of the point cloud according to the distance from tge viewpoint
//  typedef typename ViewpointDistance<PointT>::ConstIterator ConstIterator;
//  ViewpointDistance<PointT> sorter;
//  sorter.rayo = rayo;
//  std::vector<ConstIterator> sorted;
//  for (ConstIterator it = cloud.begin(); it != cloud.end(); ++it) {
//    sorted.push_back(it);
//  }
//  std::sort(sorted.begin(),sorted.end(),sorter);  

//  // Viewpoint coordinates and direction
//  rayo = viewpoint_.translation();
//  std::set<int> viewed;
//  castRays(rayds);
//  for (int i = 0; i < (int)rayds.size(); ++i) {
//    // Intersects all the boxes
//    int idxmin = -1;
//    float distMin = 1e+6;
//    bool inters = false;
//    for (int ii = 0; ii < (int)sorted.size() && !inters; ++ii) {
//      ConstIterator it = sorted[ii];
//      bctr <<  it->x, it->y, it->z;
//      bmin << it->x - res_/2, it->y - res_/2, it->z - res_/2;
//      bmax << it->x + res_/2, it->y + res_/2, it->z + res_/2;
//      inters = intersectRayBox(rayo,rayds[i],bmin,bmax,rayi);
//      float dist = (rayi-rayo).norm();
//      if (inters && dist < distMin) {
//        idxmin = std::distance(cloud.begin(),it);
//        distMin = dist;
//      }
//    }
//    //std::cout << __FILE__ << "," << __LINE__ << ": ray " << i << " intersect box " << idxmin << std::endl;
//    if (idxmin>=0 && viewed.find(idxmin) == viewed.end()) {
//      indices.indices.push_back(idxmin);
//      viewed.insert(idxmin);
//    }
//  }
//}

template <typename PointT>
void PinholeRayTracer::evaluate(pcl::octree::OctreePointCloudSearch<PointT>& octree_search,pcl::PointIndices& indices)
{
  Eigen::Vector3f rayo;                // ray origin
  std::vector<Eigen::Vector3f> rayds;  // ray directions (variable)

  typename pcl::PointCloud<PointT>::ConstPtr cloud = octree_search.getInputCloud();

  // Viewpoint coordinates and direction
  rayo = viewpoint_.translation();
  std::set<int> viewed;
  castRays(rayds);
  for (int i = 0; i < (int)rayds.size(); ++i) {
    // Intersects all the boxes
    std::vector<PointT,Eigen::aligned_allocator<PointT> > voxels;
    octree_search.getIntersectedVoxelCenters(rayo,rayds[i],voxels);

    std::vector<int> points_idx;
    for (int h = 0; h < (int)voxels.size(); h++) {
      std::vector<int> points_idx_temp;
      octree_search.voxelSearch(voxels[h],points_idx_temp);
      points_idx.insert(points_idx.end(),points_idx_temp.begin(),points_idx_temp.end());
    }

    int idxmin = -1;
    float distMin = 1e+6;
    for (int idx = 0; idx < (int)points_idx.size(); ++idx) {
      const PointT & pt = (*cloud)[points_idx[idx]];
      Eigen::Vector3f ept(pt.x,pt.y,pt.z);
      float dist = (ept-rayo).norm();
      if (dist < rangeMax_ && dist < distMin) {
        idxmin = points_idx[idx];
        distMin = dist;
      }
    }
    //std::cout << __FILE__ << "," << __LINE__ << ": ray " << i << " intersect box " << idxmin << std::endl;
    if (idxmin>=0 && viewed.find(idxmin) == viewed.end()) {
      indices.indices.push_back(idxmin);
      viewed.insert(idxmin);
    }
  }
}


#endif

