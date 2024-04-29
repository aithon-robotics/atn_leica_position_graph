/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "leica_position_graph/PositionStaticTransforms.h"

// ROS
#include <ros/ros.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"

namespace positiongraph_se {

PositionGraphStaticTransforms::PositionGraphStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr,
                                                     const graph_msf::StaticTransforms& staticTransforms)
    : graph_msf::StaticTransformsTf(staticTransforms) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void PositionGraphStaticTransforms::findTransformations() {
  // Print to console --------------------------
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Looking up transforms in TF-tree." << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Transforms between the following frames are required:" << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " " << lidarFrame_ << ", " << positionMeasFrame_ << ", " << rightGnssFrame_
            << ", " << bodyFrame_ << ", " << imuFrame_ << ", " << baseLinkFrame_ << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Waiting for up to 100 seconds until they arrive..." << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  // Sleep before subscribing, otherwise sometimes dying in the beginning of rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  // Imu to Cabin Link ---
  listener_.waitForTransform(imuFrame_, bodyFrame_, ros::Time(0), ros::Duration(100.0));
  listener_.lookupTransform(imuFrame_, bodyFrame_, ros::Time(0), transform);
  // I_Cabin
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, bodyFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_Cabin: " << rv_T_frame1_frame2(imuFrame_, bodyFrame_).translation() << std::endl;
  // Cabin_I
  lv_T_frame1_frame2(bodyFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, bodyFrame_).inverse();

  // Imu to Base Link ---
  listener_.waitForTransform(imuFrame_, baseLinkFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, baseLinkFrame_, ros::Time(0), transform);
  // I_Cabin
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, baseLinkFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_Base: " << rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).translation() << std::endl;
  // Cabin_I
  lv_T_frame1_frame2(baseLinkFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).inverse();

  // Imu to GNSS Left Link ---
  listener_.waitForTransform(imuFrame_, positionMeasFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, positionMeasFrame_, ros::Time(0), transform);
  // I_GnssL
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, positionMeasFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_GnssL: " << rv_T_frame1_frame2(imuFrame_, positionMeasFrame_).translation() << std::endl;
  // GnssL_I
  lv_T_frame1_frame2(positionMeasFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, positionMeasFrame_).inverse();

  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
}

}  // namespace positiongraph_se
