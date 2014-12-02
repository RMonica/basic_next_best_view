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

#ifndef RAY_TRCAER_H
#define RAY_TRACER_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/io/io.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <Eigen/Dense>

// --------------------------------------------------------
// GENERAL FUNCTIONS
// --------------------------------------------------------

/** Checks whether a ray with origin o and direction d. 
 * @param rayo origin of the ray
 * @param rayd direction of the ray
 * @param bmin minimum coordinates of the box vertices
 * @param bmax maximum coordinates of the box vertices
 */ 
bool intersectRayBox(const Eigen::Vector3f& rayo,const Eigen::Vector3f& rayd,
                     const Eigen::Vector3f& bmin,const Eigen::Vector3f& bmax,
                     Eigen::Vector3f& rayi);

/** Checks whether a ray with origin o and direction d. 
 * @param rayo origin of the ray
 * @param rayd destination point to observe
 * @param ground a vector pointing to bottom of the camera
 */
bool targetViewpoint(const Eigen::Vector3f& rayo,const Eigen::Vector3f& target,const Eigen::Vector3f& down,
                     Eigen::Affine3f& transf);

// --------------------------------------------------------
// OCCUPANCY POLICY
// Occupancy policy allows to convert the information stored inside 
// the points of a point cloud to the free/occupied/unknown information
// --------------------------------------------------------

/** Occupancy policy allows to check whether a point representing a box is free, occupied or 
 * unknown according to the conventional values of field intensity. 
 */
struct SignedIntensityOccupancyPolicy
{
  enum OccupancyStatus { FREE = 0, OCCUPIED, UNKNOWN };
  static const float EPS = 1e-4;

  template <typename PointT>
  OccupancyStatus getOccupancyStatus(const PointT& p);
};

struct ProbabilisticOccupancyPolicy
{
  enum OccupancyStatus { FREE = 0, OCCUPIED, UNKNOWN };
  static const float TOLLERANCE = 0.2;

  template <typename PointT>
  OccupancyStatus getOccupancyStatus(const PointT& p);
};

// --------------------------------------------------------
// RAY TRACING
// --------------------------------------------------------

template <typename PointT>
struct ViewpointDistance
{
  typedef typename pcl::PointCloud<PointT>::const_iterator ConstIterator;
  Eigen::Vector3f rayo;

  //ViewpointDistance(Eigen::Vector3f r,const pcl::PointCloud<PointT>& c) : rayo(r), cloud(c) { }

  bool operator()(ConstIterator it1,ConstIterator it2) 
  {
    Eigen::Vector3f v1(it1->x,it1->y,it1->z);
    Eigen::Vector3f v2(it2->x,it2->y,it2->z);
    return ((v1-rayo).norm() < (v2-rayo).norm());
  }
};

/** Class to perform ray tracing from a given viepoint.
 * The simulated camera is oriented as follows:
 * -axis Z: along the optical axis of the camera (orthogonal to image plane);
 * -axis X: pointing towards the bottom/ground;
 * -axis Y: according to right-hand rule. 
 */
//template <typename OccupancyPolicy>
class PinholeRayTracer
{
public:
  // Default values
  static const float RES = 0.02f;
  static const float RANGE_MAX = 100000.0f;
  static const float HFOV = M_PI * 160.0f / 180.0f;
  static const float VFOV = M_PI * 160.0f / 180.0f;
  static const int HSIZE = 6; //320;
  static const int VSIZE = 4; //240;

  /** Default constructor. 
   */
  PinholeRayTracer();
  
  /** Destructor. 
   */
  ~PinholeRayTracer();
  
  /** Sets the viewpoint of the ray-casting (the reference frame). 
   */
  void setViewpoint(const Eigen::Affine3f& vp);

  /** Gets the viewpoint of the ray-casting (the reference frame). 
   */
  Eigen::Affine3f getViewPoint() const;

  /** Sets the FoV angles: horizontal, along y axis, and vertical)
   */
  void setHalfFoV(float hf,float vf);

  /** Gets half horizontal FoV. 
   */
  float getHFoV() const;

  /** Gets half vertical FoV. 
   */
  float getVFoV() const;

  /** Sets the image size. 
   */
  void setImageSize(int hsize,float vsize);

  /** Gets half horizontal image size.
   */
  int getHSize() const;

  /** Gets half vertical image size.
   */
  int getVSize() const;

  /** Sets the resolution, i.e. the size of the box.
   */
  void setBoxRes(float res);

  /** Gets box resolution.
   */
  float getBoxRes() const;
  
  /** Sets maximum range of the sensor.
   */
  void setRangeMax(float res);

  /** Gets maximum range of the sensor.
   */  
  float getRangeMax() const;

  /** Creates rays.
   */
  void castRays(std::vector<Eigen::Vector3f>& rayds);

//  /** Computes the number of occupied and unknown boxes observed from the 
//   */
//  template <typename PointT>
//  void evaluate(const pcl::PointCloud<PointT>& occmap,int& occupied,int& unknown);

  /** Finds and returns the indices of the intersected boxes in the given occupancy map.
   * The occupancy map is represented by a point cloud. 
   */
  template <typename PointT>
  void evaluate(pcl::octree::OctreePointCloudSearch<PointT>& octree_search,pcl::PointIndices& indices);

  /** Prints internal status of ray caster.
   */
  friend std::ostream& operator<<(std::ostream& out,const PinholeRayTracer& prt);

private:
  Eigen::Affine3f viewpoint_;
  float hfov_;
  float vfov_;
  float hsize_;
  float vsize_;
  float res_;
  float rangeMax_;
// OccupancyPolicy occupancyChecker_;
};

// --------------------------------------------------------
// KINEMATIC FILTER
// --------------------------------------------------------

/** This KinematicFilter accepts all the possible robot configurations. 
 */
class NoKinematicFilter
{
public:
  /** Checks whether the configuration is feasible.
   */
  static bool isFeasible(const Eigen::Affine3f& transform);

  /**
   */
  static void selectFeasible(const std::vector<Eigen::Affine3f>& candidates,std::vector<Eigen::Affine3f>& feasibles);
};

#include "impl/RayTracer.hpp"

#endif

