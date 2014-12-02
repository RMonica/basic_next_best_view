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

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// STL
#include <vector>
#include <deque>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

// Custom
#include <basic_next_best_view/BasicRequest.h>
#include <basic_next_best_view/VoxelFilter.h>
#include <basic_next_best_view/BasicResponse.h>
#include <basic_next_best_view/ExecuteAction.h>
#include "basic_next_best_view.h"
#include "basic_viewpoint_evaluator.h"

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtr;

class BaseNBVHandler
{
public:
  typedef unsigned int uint;
  typedef typename actionlib::SimpleActionServer<basic_next_best_view::ExecuteAction> ExecuteActionServer;
  typedef typename boost::shared_ptr<ExecuteActionServer> ExecuteActionServerPtr;

  BaseNBVHandler(ros::NodeHandle & nh): m_nh(nh)
  {
    std::string temp_string;
    double temp_double;
    int temp_int;
    BasicViewpointEvaluator::MinMaxStep temp_minmaxstep;

    m_nh.param<std::string>(PARAM_NAME_OCCUPANCY_TOPIC,temp_string,PARAM_DEFAULT_OCCUPANCY_TOPIC);
    m_occupancy_subscriber = m_nh.subscribe(temp_string,1,&BaseNBVHandler::onOccupancyCloud,this);

    m_nh.param<std::string>(PARAM_NAME_POI_TOPIC,temp_string,PARAM_DEFAULT_POI_TOPIC);
    m_poi_subscriber = m_nh.subscribe(temp_string,1,&BaseNBVHandler::onPOI,this);

    m_nh.param<std::string>(PARAM_NAME_VOXEL_FILTER_TOPIC,temp_string,PARAM_DEFAULT_VOXEL_FILTER_TOPIC);
    m_voxel_filter_subscriber = m_nh.subscribe(temp_string,1,&BaseNBVHandler::onVoxelFilter,this);

    m_nh.param<double>(PARAM_NAME_VOXEL_RESOLUTION,temp_double,PARAM_DEFAULT_VOXEL_RESOLUTION);
    m_evaluator.setVoxelResolution(temp_double);
    m_nh.param<double>(PARAM_NAME_SENSOR_RANGE_MAX,temp_double,PARAM_DEFAULT_SENSOR_RANGE_MAX);
    m_evaluator.setSensorRangeMax(temp_double);
    m_nh.param<double>(PARAM_NAME_SPHERE_RADIUS,temp_double,PARAM_DEFAULT_SPHERE_RADIUS);
    m_evaluator.setSphereRadius(temp_double);

    m_nh.param<int>(PARAM_NAME_VERTICAL_RES,temp_int,PARAM_DEFAULT_VERTICAL_RES);
    m_evaluator.setVerticalRes(temp_int);
    m_nh.param<int>(PARAM_NAME_HORIZONTAL_RES,temp_int,PARAM_DEFAULT_HORIZONTAL_RES);
    m_evaluator.setHorizontalRes(temp_int);

    m_nh.param<double>(PARAM_NAME_HALF_FOV_HOR,temp_double,PARAM_DEFAULT_HALF_FOV_HOR);
    m_evaluator.setHalfHorizontalFOV(temp_double * M_PI / 180.0);
    m_nh.param<double>(PARAM_NAME_HALF_FOV_VER,temp_double,PARAM_DEFAULT_HALF_FOV_VER);
    m_evaluator.setHalfVerticalFOV(temp_double * M_PI / 180.0);

    m_nh.param<double>(PARAM_NAME_LATITUDE_MAX,temp_double,PARAM_DEFAULT_LATITUDE_MAX);
    temp_minmaxstep.max = temp_double * M_PI / 180.0;
    m_nh.param<double>(PARAM_NAME_LATITUDE_MIN,temp_double,PARAM_DEFAULT_LATITUDE_MIN);
    temp_minmaxstep.min = temp_double * M_PI / 180.0;
    m_nh.param<double>(PARAM_NAME_LATITUDE_STEP,temp_double,PARAM_DEFAULT_LATITUDE_STEP);
    temp_minmaxstep.step = temp_double * M_PI / 180.0;
    m_evaluator.setLatitudeSampling(temp_minmaxstep);

    m_nh.param<double>(PARAM_NAME_LONGITUDE_MAX,temp_double,PARAM_DEFAULT_LONGITUDE_MAX);
    temp_minmaxstep.max = temp_double * M_PI / 180.0;
    m_nh.param<double>(PARAM_NAME_LONGITUDE_MIN,temp_double,PARAM_DEFAULT_LONGITUDE_MIN);
    temp_minmaxstep.min = temp_double * M_PI / 180.0;
    m_nh.param<double>(PARAM_NAME_LONGITUDE_STEP,temp_double,PARAM_DEFAULT_LONGITUDE_STEP);
    temp_minmaxstep.step = temp_double * M_PI / 180.0;
    m_evaluator.setLongitudeSampling(temp_minmaxstep);

    m_nh.param<double>(PARAM_NAME_ZROTATION_MAX,temp_double,PARAM_DEFAULT_ZROTATION_MAX);
    temp_minmaxstep.max = temp_double * M_PI / 180.0;
    m_nh.param<double>(PARAM_NAME_ZROTATION_MIN,temp_double,PARAM_DEFAULT_ZROTATION_MIN);
    temp_minmaxstep.min = temp_double * M_PI / 180.0;
    m_nh.param<double>(PARAM_NAME_ZROTATION_STEP,temp_double,PARAM_DEFAULT_ZROTATION_STEP);
    temp_minmaxstep.step = temp_double * M_PI / 180.0;
    m_evaluator.setZRotationSampling(temp_minmaxstep);

    m_nh.param<std::string>(PARAM_NAME_EXECUTE_ACTION,temp_string,PARAM_DEFAULT_EXECUTE_ACTION);
    m_execute_action_server = ExecuteActionServerPtr(
      new ExecuteActionServer(m_nh,temp_string,boost::bind(&BaseNBVHandler::onExecuteAction,this,_1),false));

    m_nh.param<std::string>(PARAM_NAME_WORLD_FRAME,m_world_frame,PARAM_DEFAULT_WORLD_FRAME);

    m_nh.param<std::string>(PARAM_NAME_ACK_TOPIC,temp_string,PARAM_DEFAULT_ACK_TOPIC);
    m_ack_publisher = m_nh.advertise<std_msgs::Empty>(temp_string,10);

    m_nh.param<double>(PARAM_NAME_UNKNOWN_WEIGHT,temp_double,PARAM_DEFAULT_UNKNOWN_WEIGHT);
    m_evaluator.setUnknownWeight(temp_double);
    m_nh.param<double>(PARAM_NAME_OCCUPIED_WEIGHT,temp_double,PARAM_DEFAULT_OCCUPIED_WEIGHT);
    m_evaluator.setOccupiedWeight(temp_double);

    m_evaluator.setTerminationCondition(*this);

    // scope only
    {
      boost::mutex::scoped_lock lock(m_worker_mutex);
      m_is_terminated = false;
      m_is_running = false;
    }

    m_execute_action_server->start();
  }

  virtual ~BaseNBVHandler()
  {
    // scope only
    {
      boost::mutex::scoped_lock lock(m_worker_mutex);
      m_is_terminated = true;
    }

    if (m_execute_action_server)
      m_execute_action_server->shutdown();
  }

  /// Callback to receive the occupancy grid computed as a point cloud
  void onOccupancyCloud(const sensor_msgs::PointCloud2 & cloud_msg)
  {
    ROS_INFO("basic_next_best_view: occupancy cloud received.");

    boost::mutex::scoped_lock lock(m_worker_mutex);
    if (m_is_running)
    {
      ROS_ERROR("basic_next_best_view: attempted to change parameters while running!");
      return;
    }

    BasicViewpointEvaluator::PointCloudXYZI::Ptr cloud =
      BasicViewpointEvaluator::PointCloudXYZI::Ptr(new BasicViewpointEvaluator::PointCloudXYZI);
    pcl::fromROSMsg(cloud_msg,*cloud);
    m_evaluator.setOccupancyCloud(cloud);

    m_ack_publisher.publish(std_msgs::Empty());
  }

  void onPOI(const geometry_msgs::Point & pt)
  {
    ROS_INFO("basic_next_best_view: POI received.");
    boost::mutex::scoped_lock lock(m_worker_mutex);
    if (m_is_running)
    {
      ROS_ERROR("basic_next_best_view: attempted to change parameters while running!");
      return;
    }

    Eigen::Vector3f poi;
    poi.x() = pt.x;
    poi.y() = pt.y;
    poi.z() = pt.z;
    m_evaluator.setPOI(poi);

    m_ack_publisher.publish(std_msgs::Empty());
  }

  void onCandidates(const geometry_msgs::PoseArray & poses)
  {
    ROS_INFO("basic_next_best_view: Candidates received.");
    boost::mutex::scoped_lock lock(m_worker_mutex);
    if (m_is_running)
    {
      ROS_ERROR("basic_next_best_view: attempted to change parameters while running!");
      return;
    }

    std::vector<Eigen::Affine3f> sensor_poses;
    for (uint i = 0; i < poses.poses.size(); i++)
      sensor_poses.push_back(_FromROSPoseToEigen(poses.poses[i]));
    m_evaluator.setSensorPoses(sensor_poses);

    m_ack_publisher.publish(std_msgs::Empty());
  }

  class TBBoxVoxelFilter: public BasicViewpointEvaluator::IVoxelFilter
  {
    public:
    TBBoxVoxelFilter(const Eigen::Vector3f & bbox_min,const Eigen::Vector3f & bbox_max):
      m_bbox_min(bbox_min),m_bbox_max(bbox_max) {}

    virtual bool Evaluate(BasicViewpointEvaluator::PointCloudXYZI::Ptr cloud,uint pidx,
      const Eigen::Affine3f & /*viewpoint*/)
    {
      const pcl::PointXYZI & pt = (*cloud)[pidx];
      Eigen::Vector3f ept(pt.x,pt.y,pt.z);
      return ept.x() <= m_bbox_max.x() && ept.y() <= m_bbox_max.y() && ept.z() <= m_bbox_max.z() &&
        ept.x() >= m_bbox_min.x() && ept.y() >= m_bbox_min.y() && ept.z() >= m_bbox_min.z();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    private:
    Eigen::Vector3f m_bbox_min,m_bbox_max;
  };

  class TSphereVoxelFilter: public BasicViewpointEvaluator::IVoxelFilter
  {
    public:
    TSphereVoxelFilter(const Eigen::Vector3f & center,float radius):
      m_center(center),m_radius(radius) {}

    virtual bool Evaluate(BasicViewpointEvaluator::PointCloudXYZI::Ptr cloud,uint pidx,
      const Eigen::Affine3f & /*viewpoint*/)
    {
      const pcl::PointXYZI & pt = (*cloud)[pidx];
      Eigen::Vector3f ept(pt.x,pt.y,pt.z);
      return (ept - m_center).norm() < m_radius;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    private:
    Eigen::Vector3f m_center;
    float m_radius;
  };

  void onVoxelFilter(const basic_next_best_view::VoxelFilter & filter)
  {
    ROS_INFO("basic_next_best_view: voxel filter received.");
    boost::mutex::scoped_lock lock(m_worker_mutex);
    if (m_is_running)
    {
      ROS_ERROR("basic_next_best_view: attempted to change parameters while running!");
      return;
    }

    if (filter.type == filter.FILTER_NONE)
    {
      ROS_INFO("basic_next_best_view: voxel filter disabled.");
      m_evaluator.setVoxelFilter(BasicViewpointEvaluator::IVoxelFilter::Ptr());
      m_ack_publisher.publish(std_msgs::Empty());
    }
    else if (filter.type == filter.FILTER_BOUNDING_BOX)
    {
      ROS_INFO("basic_next_best_view: voxel filter: bounding box.");
      Eigen::Vector3f bbox_min,bbox_max;
      for (uint i = 0; i < 3; i++)
        bbox_min[i] = filter.bbox_min[i];
      for (uint i = 0; i < 3; i++)
        bbox_max[i] = filter.bbox_max[i];

      m_evaluator.setVoxelFilter(BasicViewpointEvaluator::IVoxelFilter::Ptr(
        new TBBoxVoxelFilter(bbox_min,bbox_max)));

      m_ack_publisher.publish(std_msgs::Empty());
    }
    else if (filter.type == filter.FILTER_SPHERE)
    {
      ROS_INFO("basic_next_best_view: voxel filter: sphere.");
      Eigen::Vector3f center;
      for (uint i = 0; i < 3; i++)
        center[i] = filter.center[i];
      float radius = filter.radius;
      m_evaluator.setVoxelFilter(BasicViewpointEvaluator::IVoxelFilter::Ptr(
        new TSphereVoxelFilter(center,radius)));

      m_ack_publisher.publish(std_msgs::Empty());
    }
    else
      ROS_ERROR("basic_next_best_view: unknown voxel filter type: %u",uint(filter.type));
  }

  void onExecuteAction(basic_next_best_view::ExecuteGoalConstPtr goal)
  {
    const basic_next_best_view::BasicRequest & request = goal->request;

    // scope only
    {
      boost::mutex::scoped_lock lock(m_worker_mutex);
      if (m_is_running)
      {
        ROS_ERROR("basic_next_best_view: Received request, but already running!");
        m_execute_action_server->setAborted();
        return;
      }

      if (m_is_terminated)
      {
        m_execute_action_server->setAborted();
        return;
      }

      m_is_running = true;
    }

    basic_next_best_view::ExecuteResultPtr result(new basic_next_best_view::ExecuteResult);
    basic_next_best_view::BasicResponse & response = result->response;
    response.ok = false;

    if (request.type == request.REQUEST_NOOP)
      requestNoopWorker(request,response);
    else if (request.type == request.REQUEST_GET_POSE_LIST)
      requestPoseListWorker(request,response);
    else if (request.type == request.REQUEST_GET_BEST_INDICES)
      requestBestIndicesWorker(request,response);
    else if (request.type == request.REQUEST_GET_BEST_POSES)
      requestBestPosesWorker(request,response);
    else if (request.type == request.REQUEST_GET_POSE_WEIGHTS)
      requestPoseWeightWorker(request,response);
    else if (request.type == request.REQUEST_GET_POSE_COUNT)
      requestPoseCountWorker(request,response);
    else if (request.type == request.REQUEST_GET_UNK_COUNT)
      requestUnknownCountWorker(request,response);
    else
      ROS_ERROR("basic_next_best_view: unknown request type: %u.",uint(request.type));

    if (response.ok)
      m_execute_action_server->setSucceeded(*result);
      else
        m_execute_action_server->setAborted(*result);

    // scope only
    {
      boost::mutex::scoped_lock lock(m_worker_mutex);
      m_is_running = false;
    }
  }

  void requestNoopWorker(const basic_next_best_view::BasicRequest & request,basic_next_best_view::BasicResponse & response)
  {
    ROS_INFO("basic_next_best_view: noop.");
    response.ok = true;
  }

  void requestPoseListWorker(const basic_next_best_view::BasicRequest & request,basic_next_best_view::BasicResponse & response)
  {
    ROS_INFO("basic_next_best_view: pose list.");

    const std::vector<Eigen::Affine3f> * sensor_poses = m_evaluator.getCandidates();

    if (isTerminated())
      return;

    if (!sensor_poses)
    {
      ROS_ERROR("basic_next_best_view: Couldn't compute poses!");
      return;
    }

    geometry_msgs::PoseArray & pose_array = response.pose_list;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = m_world_frame;

    uint size = (*sensor_poses).size();
    pose_array.poses.reserve(size);
    for (uint i = 0; i < size; i++)
      {
      // apply index filter
      if (request.has_index_range && (i < request.start_from_index || i >= request.end_before_index))
        continue;

      pose_array.poses.push_back(_FromEigenToROSPose((*sensor_poses)[i]));
      }

    response.ok = true;
  }

  void requestPoseCountWorker(const basic_next_best_view::BasicRequest & request,basic_next_best_view::BasicResponse & response)
  {
    ROS_INFO("basic_next_best_view: pose count.");

    const std::vector<Eigen::Affine3f> * sensor_poses = m_evaluator.getCandidates();

    if (isTerminated())
      return;

    if (!sensor_poses)
    {
      ROS_ERROR("basic_next_best_view: Couldn't compute poses!");
      return;
    }

    uint count = sensor_poses->size();

    response.pose_count = count;
    response.ok = true;
  }

  void requestBestIndicesWorker(const basic_next_best_view::BasicRequest & request,basic_next_best_view::BasicResponse & response)
  {
    ROS_INFO("basic_next_best_view: best indices.");

    const std::vector<uint> * indices = m_evaluator.getOrderedPoseIndices();

    if (isTerminated())
      return;

    if (!indices)
    {
      ROS_ERROR("basic_next_best_view: Couldn't order poses!");
      return;
    }

    std_msgs::UInt32MultiArray & message = response.index_list;

    uint size = indices->size();
    uint counter = 0;
    message.data.reserve(size);
    for (uint i = 0; i < size; i++)
      {
      // apply index filter
      if (request.has_index_range && (i < request.start_from_index || i >= request.end_before_index))
        continue;

      message.data.push_back((*indices)[i]);
      counter++;
      }

    std_msgs::MultiArrayDimension dim;
    dim.label = "indices";
    dim.size = counter;
    dim.stride = 1;
    message.layout.data_offset = 0;
    message.layout.dim.push_back(dim);

    response.ok = true;
  }

  void requestBestPosesWorker(const basic_next_best_view::BasicRequest & request,basic_next_best_view::BasicResponse & response)
  {
    ROS_INFO("basic_next_best_view: best poses.");

    const std::vector<Eigen::Affine3f> * sensor_poses = m_evaluator.getCandidates();
    const std::vector<uint> * indices = m_evaluator.getOrderedPoseIndices();

    if (isTerminated())
      return;

    if (!sensor_poses || !indices)
    {
      ROS_ERROR("basic_next_best_view: Couldn't compute weights!");
      return;
    }

    geometry_msgs::PoseArray & pose_array = response.best_poses;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = m_world_frame;

    uint size = indices->size();
    pose_array.poses.reserve(size);
    for (uint i = 0; i < size; i++)
      {
      // apply index filter
      if (request.has_index_range && (i < request.start_from_index || i >= request.end_before_index))
        continue;

      pose_array.poses.push_back(_FromEigenToROSPose((*sensor_poses)[(*indices)[i]]));
      }

    response.ok = true;
  }

  void requestPoseWeightWorker(const basic_next_best_view::BasicRequest & request,basic_next_best_view::BasicResponse & response)
  {
    ROS_INFO("basic_next_best_view: pose weights.");

    const std::vector<BasicViewpointEvaluator::Weight> * weights = m_evaluator.getWeights();

    if (isTerminated())
      return;

    if (!weights)
    {
      ROS_ERROR("basic_next_best_view: Couldn't compute weights!");
      return;
    }

    std_msgs::Float64MultiArray & message = response.weight_list;

    uint size = (*weights).size();
    uint counter = 0;
    message.data.reserve(size);
    for (uint i = 0; i < size; i++)
      {
      // apply index filter
      if (request.has_index_range && (i < request.start_from_index || i >= request.end_before_index))
        continue;

      message.data.push_back((*weights)[i]);
      counter++;
      }

    std_msgs::MultiArrayDimension dim;
    dim.label = "weights";
    dim.size = counter;
    dim.stride = 1;
    message.layout.data_offset = 0;
    message.layout.dim.push_back(dim);

    response.ok = true;
  }

  void requestUnknownCountWorker(const basic_next_best_view::BasicRequest & request,basic_next_best_view::BasicResponse & response)
  {
    ROS_INFO("basic_next_best_view: pose weights.");

    BasicViewpointEvaluator::UintVectorConstPtr unknown = m_evaluator.getUnknownCount();

    if (isTerminated())
      return;

    if (!unknown)
    {
      ROS_ERROR("basic_next_best_view: Couldn't compute unknown!");
      return;
    }

    std_msgs::UInt32MultiArray & message = response.unknown_count;

    uint size = unknown->size();
    uint counter = 0;
    message.data.reserve(size);
    for (uint i = 0; i < size; i++)
      {
      // apply index filter
      if (request.has_index_range && (i < request.start_from_index || i >= request.end_before_index))
        continue;

      message.data.push_back((*unknown)[i]);
      counter++;
      }

    std_msgs::MultiArrayDimension dim;
    dim.label = "unknown_count";
    dim.size = counter;
    dim.stride = 1;
    message.layout.data_offset = 0;
    message.layout.dim.push_back(dim);

    response.ok = true;
  }

  bool isTerminated()
  {
    boost::mutex::scoped_lock lock(m_worker_mutex);
    return m_is_terminated;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  std::deque<basic_next_best_view::BasicRequest> m_request_queue;

  BasicViewpointEvaluator m_evaluator;

  ExecuteActionServerPtr m_execute_action_server;

  ros::NodeHandle & m_nh;
  ros::Subscriber m_occupancy_subscriber;
  ros::Subscriber m_poi_subscriber;
  ros::Subscriber m_candidates_subscriber;
  ros::Subscriber m_voxel_filter_subscriber;
  ros::Publisher m_ack_publisher;

  std::string m_world_frame;

  bool m_is_terminated;
  bool m_is_running;
  boost::mutex m_worker_mutex;

  geometry_msgs::Pose _FromEigenToROSPose(const Eigen::Affine3f & epose)
  {
    geometry_msgs::Pose rpose;
    tf::poseEigenToMsg(epose.cast<double>(),rpose);
    return rpose;
  }

  Eigen::Affine3f _FromROSPoseToEigen(const geometry_msgs::Pose & rpose)
  {
    Eigen::Affine3d result;
    tf::poseMsgToEigen(rpose,result);
    return result.cast<float>();
  }
};

int main(int argc,char** argv)
{
  ros::init (argc, argv, "nbv_finder");

  // Reading parameters
  ros::NodeHandle nh("~");
  BaseNBVHandler nbv(nh);

  ros::spin();

  return 0;
}
