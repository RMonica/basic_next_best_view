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

#include "basic_viewpoint_evaluator.h"

BasicViewpointEvaluator::BasicViewpointEvaluator()
{
  m_poi_received = false;
  m_occupancy_cloud_received = false;
  m_sensor_poses_computed = false;
  m_pose_weights_computed = false;
  m_ordered_pose_indices_computed = false;

  m_unknown_weight = 1.0;
  m_occupied_weight = 0.0;
}

void BasicViewpointEvaluator::computeCandidates()
{
  if (!m_poi_received)
  {
    ROS_ERROR("basic_next_best_view: POI not set. Can't continue!");
    return;
  }

  if (m_sensor_poses_computed)
    return; // be lazy

  m_sensor_poses.clear();

  // scope only
  {
    Eigen::Vector3f pos;
    Eigen::Vector3f target;
    Eigen::Vector3f down;
    Eigen::Affine3f transf;
    Eigen::Affine3f center = Eigen::Affine3f::Identity();
    center.translate(m_poi);
    int tmin = (int)round(m_latitude.min / m_latitude.step);
    int tmax = (int)round(m_latitude.max / m_latitude.step);
    int pmin = (int)round(m_longitude.min / m_longitude.step);
    int pmax = (int)round(m_longitude.max / m_longitude.step);
    int zmin = (int)round(m_zrotation.min / m_zrotation.step);
    int zmax = (int)round(m_zrotation.max / m_zrotation.step);
    target << 0.0f, 0.0f, 0.0f;
    for (int t = tmin; t <= tmax; ++t) {
      for (int p = pmin; p <= pmax; ++p) {
        for (int rz = zmin; rz <= zmax; ++rz) {
          float theta = m_latitude.step * t;
          float phi = m_longitude.step * p;
          float radius_rotation = rz * m_zrotation.step; // rotation around the selected radius
          // Computes the position of candidate viewpoint sampled on a sphere
          pos << m_sphere_radius * sin(theta) * cos(phi),
                 m_sphere_radius * sin(theta) * sin(phi),
                 m_sphere_radius * cos(theta);
          down <<  cos(theta) * cos(phi),
                   cos(theta) * sin(phi),
                  -sin(theta);
          // rotate the down vector by rz around pos-target
          down = Eigen::AngleAxisf(radius_rotation,(pos - target).normalized()) * down;
          bool valid = targetViewpoint(pos,target,down,transf);
          if (valid) {
            // Moves the viewpoint to the center of the observation
            transf = center * transf;
            m_sensor_poses.push_back(transf);
          }

        if (_IsTerminated())
          return;
        }
      }
    }
  }

  m_sensor_poses_computed = true;
}

void BasicViewpointEvaluator::computeWeights()
{
  if (!m_occupancy_cloud_received)
  {
    ROS_ERROR("basic_next_best_view: occupancy cloud not set. Can't continue!");
    return;
  }

  computeCandidates();

  if (_IsTerminated())
    return;

  if (!m_sensor_poses_computed)
  {
    ROS_ERROR("basic_next_best_view: couldn't compute candidates!");
    return;
  }

  if (m_pose_weights_computed)
    return; // be lazy

  m_simulated_poses.clear();
  m_pose_weights.clear();

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree_search(m_raytracer.getBoxRes());
  {
    octree_search.setInputCloud(m_occupancy_cloud);
    octree_search.addPointsFromInputCloud();
  }

  m_simulated_poses.reserve(m_sensor_poses.size());
  m_pose_weights.reserve(m_sensor_poses.size());

  for (int i = 0; i < (int)m_sensor_poses.size(); ++i)
  {
    //std::cout << __FILE__ << "," << __LINE__ << ": " << i << " of " << m_sensor_poses.size() << " candidate viewpoints to assess" << std::endl;
    CandidateViewpoint cvp;
    cvp.viewpoint = m_sensor_poses[i];
    pcl::PointIndices indices;
    m_raytracer.setViewpoint(m_sensor_poses[i]);
    m_raytracer.evaluate(octree_search,indices);

    if (_IsTerminated())
      return;

    // Counts the occupied and unknown boxes
    cvp.occupied = 0;
    cvp.unknown = 0;
    std::vector<int>::iterator it = indices.indices.begin();
    for (; it != indices.indices.end(); ++it) {
      if (0 <= *it && *it < (int)(*m_occupancy_cloud).size()) {
        if (!m_voxel_filter || m_voxel_filter->Evaluate(m_occupancy_cloud,*it,m_sensor_poses[i])) {
          if (m_occupancy_policy.getOccupancyStatus((*m_occupancy_cloud).points[*it])
            == SignedIntensityOccupancyPolicy::OCCUPIED) {
            cvp.occupied++;
          }
          else if (m_occupancy_policy.getOccupancyStatus((*m_occupancy_cloud).points[*it])
            == SignedIntensityOccupancyPolicy::UNKNOWN) {
            cvp.unknown++;
          }
        }
      }
      else {
        std::cerr << __FILE__ << "," << __LINE__ << ": invalid index " << *it << ": max size "
          << (*m_occupancy_cloud).size() << std::endl;
      }

    if (_IsTerminated())
      return;
    }
    // Inserts viewpoint with statistics
    m_simulated_poses.push_back(cvp);
    m_pose_weights.push_back(computeWeight(cvp.unknown,cvp.occupied));
  }

  m_pose_weights_computed = true;
}

void BasicViewpointEvaluator::orderWeights()
{
  computeWeights();

  if (_IsTerminated())
    return;

  if (!m_pose_weights_computed)
  {
    ROS_ERROR("basic_next_best_view: Couldn't compute weights! Can't continue!");
    return;
  }

  m_ordered_pose_indices.resize(m_pose_weights.size());
  for (uint i = 0; i < m_ordered_pose_indices.size(); i++)
    m_ordered_pose_indices[i] = i;

  std::sort(m_ordered_pose_indices.begin(),m_ordered_pose_indices.end(),ByWeightCompare(m_pose_weights));

  m_ordered_pose_indices_computed = true;
}
