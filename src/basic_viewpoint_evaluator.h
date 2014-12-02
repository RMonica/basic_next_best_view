/*
 * Copyright (c) 2013-2014, Riccardo Monica
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

#ifndef BASIC_VIEWPOINT_EVALUATOR_H
#define BASIC_VIEWPOINT_EVALUATOR_H

// STL
#include <vector>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3f);

// ROS
#include <ros/ros.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// custom
#include "RayTracer.h"

class BasicViewpointEvaluator
{
public:
  typedef unsigned int uint;
  typedef unsigned int PoseIndex;
  typedef typename std::vector<PoseIndex> PoseIndexVector;
  typedef double Weight;
  typedef typename pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
  typedef typename std::vector<uint> UintVector;
  typedef typename boost::shared_ptr<const UintVector> UintVectorConstPtr;
  typedef typename boost::shared_ptr<UintVector> UintVectorPtr;

  struct CandidateViewpoint
  {
    Eigen::Affine3f viewpoint;
    int occupied;
    int unknown;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  typedef std::vector<CandidateViewpoint,Eigen::aligned_allocator<CandidateViewpoint> > CandidateViewpointVector;

  struct MinMaxStep
  {
    double min,max,step;
  };

  BasicViewpointEvaluator();

  void computeCandidates();
  void computeWeights();
  void orderWeights();

  void setVoxelResolution(double res)
  {
    m_voxel_resolution = res;
    m_raytracer.setBoxRes(res);
  }

  void setSensorRangeMax(double max)
  {
    m_sensor_range_max = max;
    m_raytracer.setRangeMax(max);
  }

  void setSphereRadius(double radius)
  {
    m_sphere_radius = radius;
  }

  void setVerticalRes(int res)
  {
    m_vertical_res = std::max(1,res);
    m_raytracer.setImageSize(m_vertical_res,m_horizontal_res);
  }

  void setHorizontalRes(int res)
  {
    m_horizontal_res = std::max(1,res);
    m_raytracer.setImageSize(m_vertical_res,m_horizontal_res);
  }

  void setHalfHorizontalFOV(double fov)
  {
    m_hhfov = fov;
    m_raytracer.setHalfFoV(m_hhfov,m_hvfov);
  }

  void setHalfVerticalFOV(double fov)
  {
    m_hvfov = fov;
    m_raytracer.setHalfFoV(m_hhfov,m_hvfov);
  }

  void setLatitudeSampling(const MinMaxStep & sampling)
  {
    m_latitude = sampling;
  }

  void setLongitudeSampling(const MinMaxStep & sampling)
  {
    m_longitude = sampling;
  }

  void setZRotationSampling(const MinMaxStep & sampling)
  {
    m_zrotation = sampling;
  }

  void setOccupancyCloud(PointCloudXYZI::Ptr pc)
  {
    m_occupancy_cloud = pc;
    m_occupancy_cloud_received = true;

    m_pose_weights_computed = false;
    m_ordered_pose_indices_computed = false;
  }

  void setPOI(const Eigen::Vector3f & poi)
  {
    m_poi = poi;
    m_poi_received = true;

    m_sensor_poses_computed = false;
    m_pose_weights_computed = false;
    m_ordered_pose_indices_computed = false;
  }

  void setSensorPoses(const std::vector<Eigen::Affine3f> & poses)
  {
    m_sensor_poses = poses;

    m_sensor_poses_computed = true;
    m_pose_weights_computed = false;
    m_ordered_pose_indices_computed = false;
  }

  class IVoxelFilter
  {
    public:
    typedef boost::shared_ptr<IVoxelFilter> Ptr;
    /// checks if a voxel is a valid voxel
    /// @param pidx the index in the occupancy cloud
    /// @param viewpoint the viewpoint that is seeing the voxel
    /// @return true if the evaluator should count the voxel
    virtual bool Evaluate(PointCloudXYZI::Ptr cloud,uint pidx,const Eigen::Affine3f & viewpoint) = 0;

    virtual ~IVoxelFilter() {}
  };

  // whenever filter.Evaluate returns false, the voxel is excluded
  // from the voxel count
  // set to Ptr() (=NULL) to disable
  void setVoxelFilter(IVoxelFilter::Ptr filter)
  {
    m_voxel_filter = filter;

    m_pose_weights_computed = false;
    m_ordered_pose_indices_computed = false;
  }

  bool isSensorPosesComputed() { return m_sensor_poses_computed;}
  bool isPoseWeightsComputed() { return m_pose_weights_computed;}
  bool isOrderedPosesComputed() { return m_ordered_pose_indices_computed;}

  // NULL if failed
  const std::vector<Eigen::Affine3f> * getCandidates()
  {
    computeCandidates();

    if (_IsTerminated() || !m_sensor_poses_computed)
      return NULL;

    return &m_sensor_poses;
  }

  // NULL if failed
  const std::vector<uint> * getOrderedPoseIndices()
  {
    orderWeights();

    if (_IsTerminated() || !m_ordered_pose_indices_computed)
      return NULL;

    return &m_ordered_pose_indices;
  }

  // NULL if failed
  const std::vector<Weight> * getWeights()
  {
    computeWeights();

    if (_IsTerminated() || !m_pose_weights_computed)
      return NULL;

    return &m_pose_weights;
  }

  // NULL if failed
  const UintVectorConstPtr getUnknownCount()
  {
    computeWeights();

    if (_IsTerminated() || !m_pose_weights_computed)
      return UintVectorConstPtr();

    UintVectorPtr result = UintVectorPtr(new UintVector);

    result->reserve(m_simulated_poses.size());
    for (uint i = 0;i < m_simulated_poses.size(); i++)
      result->push_back(m_simulated_poses[i].unknown);

    return result;
  }

  void setOccupiedWeight(double w) {m_occupied_weight = w; }
  void setUnknownWeight(double w) {m_unknown_weight = w; }

  Weight computeWeight(uint unknown,uint occupied) const
  {
    return Weight(m_unknown_weight * unknown + m_occupied_weight * occupied);
  }

  struct ByWeightCompare
  {
    const std::vector<Weight> & weights;
    ByWeightCompare(const std::vector<Weight> & w): weights(w) {}
    bool operator()(PoseIndex l,PoseIndex r) const
    {
      return weights[l] > weights[r];
    }
  };

  template <class T>
  // set a reference to any class with a method named "isTerminated()"
  void setTerminationCondition(T & obj)
  {
    m_termination_condition = TerminationCondition::Ptr(new TerminationConditionAdapter<T>(obj));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  bool _IsTerminated()
  {
    return (bool(m_termination_condition) && m_termination_condition->isTerminated());
  }

  PointCloudXYZI::Ptr m_occupancy_cloud;
    // the occupancy cloud, with intensity:
    // < 0, no point: empty
    // > 0: occupied
    // =~ 0: unknown
  bool m_occupancy_cloud_received;

  Eigen::Vector3f m_poi; // the sensor will point towards the Point Of Interest
  bool m_poi_received;

  std::vector<Eigen::Affine3f> m_sensor_poses;
  bool m_sensor_poses_computed;

  CandidateViewpointVector m_simulated_poses;
  std::vector<Weight> m_pose_weights;
  bool m_pose_weights_computed;

  PoseIndexVector m_ordered_pose_indices;
  bool m_ordered_pose_indices_computed;

  PinholeRayTracer m_raytracer;
  SignedIntensityOccupancyPolicy m_occupancy_policy;

  double m_voxel_resolution;
  double m_sensor_range_max;
  double m_sphere_radius;

  uint m_vertical_res;
  uint m_horizontal_res;

  double m_hhfov;
  double m_hvfov;

  MinMaxStep m_zrotation;
  MinMaxStep m_latitude;
  MinMaxStep m_longitude;

  double m_occupied_weight;
  double m_unknown_weight;

  class TerminationCondition
  {
  public:
    virtual ~TerminationCondition() {}
    virtual bool isTerminated() = 0;

    typedef boost::shared_ptr<TerminationCondition> Ptr;
  };

  TerminationCondition::Ptr m_termination_condition;

  IVoxelFilter::Ptr m_voxel_filter;

  template <class T>
  class TerminationConditionAdapter: public TerminationCondition
  {
  public:
    TerminationConditionAdapter(T & obj): m_instance(obj) {}
    bool isTerminated() {return m_instance.isTerminated(); }

    T & m_instance;
  };
};

#endif // BASIC_VIEWPOINT_EVALUATOR_H
