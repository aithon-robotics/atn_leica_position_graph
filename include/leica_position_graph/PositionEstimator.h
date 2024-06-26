/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

// std
#include <chrono>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>

// Leica Position extension
#include <geometry_msgs/PointStamped.h>

// Workspace
#include "leica_position_graph/PositionStaticTransforms.h"
#include "graph_msf/gnss/GnssHandler.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/GraphMsfRos.h"

// Defined Macros
#define NUM_POSITION_CALLBACKS_UNTIL_START 20
#define POSITION_MEAS_POSITION_COVARIANCE_VIOLATION_THRESHOLD 0.2  // 10000
#define POSITION_MEAS_HEADING_COVARIANCE_VIOLATION_THRESHOLD 1.0   // 10000

namespace positiongraph_se {

class PositionGraphEstimator : public graph_msf::GraphMsfRos {
 public:
  PositionGraphEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr);
  // Destructor
  ~PositionGraphEstimator() = default;
  // Setup
  virtual bool setup() override;

 private:
  // Methods ------------------------------------
  void initializePublishers_(ros::NodeHandle& privateNode) override;

  void initializeSubscribers_(ros::NodeHandle& privateNode) override;

  void initializeMessages_(ros::NodeHandle& privateNode) override;

  void readParams_(const ros::NodeHandle& privateNode) override;

  // Callbacks
  void positionCallback_(const geometry_msgs::PointStamped::ConstPtr& LeicaPositionPtr);
  void realsenseOdometryCallback_(const nav_msgs::Odometry::ConstPtr& RealsenseOdometryPtr);


  // Publish State
  virtual void publishState_(
      const std::shared_ptr<graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) override;

  // Members ----------------------------------
  // Publishers
  // Path
  ros::Publisher pubMeasWorldPositionPath_;
  ros::Publisher pubMeasMapRealsensePath_;

  // Messages
  // Paths
  nav_msgs::PathPtr measPosition_worldPositionPathPtr_;
  nav_msgs::PathPtr measRealsense_mapImuPathPtr_;

  // Subscribers
  ros::Subscriber subImu_;
  ros::Subscriber subPosition_;
  ros::Subscriber subRealsense_;
  
  tf::TransformListener tfListener_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // Rates
  double positionRate_ = 20.0;

  // Noise
  double positionMeasUnaryNoise_ = 1.0;  // in [m]
  Eigen::Matrix<double, 6, 1> RealsenseOdomPoseUnaryNoise_;

  /// Counters
  int realsenseOdometryCallbackCounter_ = 0;

};
}  // namespace positiongraph_se
#endif  // end POSITION_ESTIMATOR_H
