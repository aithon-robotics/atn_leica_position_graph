/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "leica_position_graph/PositionEstimator.h"

namespace positiongraph_se {

void PositionGraphEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  // Call super method
  graph_msf::GraphMsfRos::readParams_(privateNode);

  // Set frames ----------------------------
  /// Realsense frame
  std::string frame = graph_msf::tryGetParam<std::string>("extrinsics/RealsenseFrame", privateNode);
  dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->setRealsenseOdometryFrame(frame);

  /// Body frame
  frame = graph_msf::tryGetParam<std::string>("extrinsics/BodyFrame", privateNode);
  dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->setBodyFrame(frame);

  /// Position Measurment frame (Prism)
  frame = graph_msf::tryGetParam<std::string>("extrinsics/positionMeasFrame", privateNode);
  dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->setPositionMeasFrame(frame);

  // Sensor Parameters ----------------------------
  positionRate_ = graph_msf::tryGetParam<double>("sensor_params/positionRate", privateNode);

  /// Noise Parameters ----
  /// Position measurement unary noise
  positionMeasUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/PositionMeasUnaryNoise", privateNode);

  /// Realsense Odometry unary noise
    const auto RealsenseOdomPoseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryNoise", privateNode);  // roll,pitch,yaw,x,y,z
      RealsenseOdomPoseUnaryNoise_ << RealsenseOdomPoseUnaryNoise[0], RealsenseOdomPoseUnaryNoise[1], RealsenseOdomPoseUnaryNoise[2], RealsenseOdomPoseUnaryNoise[3], RealsenseOdomPoseUnaryNoise[4],
      RealsenseOdomPoseUnaryNoise[5];
  }

}  // namespace positiongraph_se
