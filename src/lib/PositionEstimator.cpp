/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "leica_position_graph/PositionEstimator.h"

// Project
#include "leica_position_graph/PositionStaticTransforms.h"
#include "leica_position_graph/constants.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"

namespace positiongraph_se {

PositionGraphEstimator::PositionGraphEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {

  std::cout << YELLOW_START << "LeicaPositionEstimator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static Transforms
  staticTransformsPtr_ = std::make_shared<PositionGraphStaticTransforms>(privateNodePtr);

  // GNSS Handler
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  // Setup
  if (not PositionGraphEstimator::setup()) {
    REGULAR_COUT << RED_START << " Failed to set up." << COLOR_END << std::endl;
    throw std::runtime_error("LeicaPositionEstimator could not be initialized");
  }

  std::cout << YELLOW_START << "LeicaPositionEstimator" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}


//---------------------------------------------------------------
bool PositionGraphEstimator::setup() {

  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  PositionGraphEstimator::readParams_(privateNode_);

  // Super class
  if (not graph_msf::GraphMsfRos::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Publishers ----------------------------
  PositionGraphEstimator::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  PositionGraphEstimator::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  PositionGraphEstimator::initializeMessages_(privateNode_);

  // Static Transforms
  staticTransformsPtr_->findTransformations();

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}


//---------------------------------------------------------------
void PositionGraphEstimator::initializePublishers_(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Publishers..." << COLOR_END << std::endl;

  // Paths
  pubMeasWorldPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_prism", ROS_QUEUE_SIZE);
  pubMeasMapRealsensePath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
}

void PositionGraphEstimator::initializeSubscribers_(ros::NodeHandle& privateNode) {

  subPosition_ = privateNode.subscribe<geometry_msgs::PointStamped>(
    "/position_topic", ROS_QUEUE_SIZE,  &PositionGraphEstimator::positionCallback_, this, ros::TransportHints().tcpNoDelay());

  subRealsense_ = privateNode.subscribe<nav_msgs::Odometry>(
    "/realsense_topic", ROS_QUEUE_SIZE,  &PositionGraphEstimator::realsenseOdometryCallback_, this, ros::TransportHints().tcpNoDelay());

  std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Position subscriber (on position_topic)." << std::endl;
  return;
}


//---------------------------------------------------------------
void PositionGraphEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Messages..." << COLOR_END << std::endl;

  // Paths
  measPosition_worldPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measRealsense_mapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);  // Preintegrated Estimate of LiDAR Frame
}


//---------------------------------------------------------------
void PositionGraphEstimator::positionCallback_(const geometry_msgs::PointStamped::ConstPtr& LeicaPositionPtr) {
  
  // Static variables
  static Eigen::Vector3d zeroCoord(0.0, 0.0, 0.0);
  static bool PositionHealthyFlag__ = true;
  static int positionCallbackCounter__ = 0;

  // Translate to Eigen
  Eigen::Vector3d positionCoord = Eigen::Vector3d(LeicaPositionPtr->point.x, LeicaPositionPtr->point.y, LeicaPositionPtr->point.z);
  Eigen::Vector3d positionCovarianceXYZ(positionMeasUnaryNoise_, positionMeasUnaryNoise_, positionMeasUnaryNoise_); // TODO: Set proper values

  double yaw_W_C = gnssHandlerPtr_->computeYaw(positionCoord, zeroCoord);

  // Initialize GNSS Handler
  if (positionCallbackCounter__ == 0) {  // Only measurements at the beginning
    positionCallbackCounter__++;
    gnssHandlerPtr_->initHandler(yaw_W_C);
  }

  // State Machine
  if (!areYawAndPositionInited() && areRollAndPitchInited()) {  
    // Try to initialize yaw and position (WITH ZERO POSITION) if not done already
    if (this->initYawAndPosition(yaw_W_C, positionCoord, staticTransformsPtr_->getWorldFrame(),
                                 dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->getBodyFrame(),
                                 dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->getPositionMeasFrame())) {
      REGULAR_COUT << " Set yaw and position successfully." << std::endl;
    } else {
      REGULAR_COUT << " Could not set yaw and position." << std::endl;
      }
  } else {  
    // Already initialized --> add position measurement to graph
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> meas_W_t_W_Position(
      "Leica-Position", int(positionRate_), LeicaPositionPtr->header.stamp.toSec(), staticTransformsPtr_->getWorldFrame() + "_ENU",
      dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->getPositionMeasFrame(), positionCoord, positionCovarianceXYZ,
      POSITION_MEAS_POSITION_COVARIANCE_VIOLATION_THRESHOLD);
    if (!this->addPositionMeasurement(meas_W_t_W_Position)) {
      if (PositionHealthyFlag__) {
        REGULAR_COUT << RED_START << " Position measurement not added." << COLOR_END << std::endl;
        PositionHealthyFlag__ = false;
      }
    } else if (!PositionHealthyFlag__) {
      REGULAR_COUT << GREEN_START << " Position measurement returned." << COLOR_END << std::endl;
      PositionHealthyFlag__ = true;
    }
  }

  // Visualizations
  addToPathMsg(measPosition_worldPositionPathPtr_, staticTransformsPtr_->getWorldFrame(), LeicaPositionPtr->header.stamp, positionCoord,
               graphConfigPtr_->imuBufferLength * 4);
  pubMeasWorldPositionPath_.publish(measPosition_worldPositionPathPtr_);
}

//---------------------------------------------------------------
void PositionGraphEstimator::realsenseOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomRealsensePtr) {
  
  // Counter
  ++realsenseOdometryCallbackCounter_;

  Eigen::Isometry3d lio_T_M_Lk;
  graph_msf::odomMsgToEigen(*odomRealsensePtr, lio_T_M_Lk.matrix());

  // Transform to IMU frame
  double RealsenseOdometryTimeK = odomRealsensePtr->header.stamp.toSec();

  int realsenseOdometryRateFactor = 5;  // 200 Hz -> 40 Hz
  int realSenseOdometryRate = 200/realsenseOdometryRateFactor;

  if (realsenseOdometryCallbackCounter_ <= (realsenseOdometryRateFactor-1)) { // only every 5th message is used 200Hz -> 40Hz
    return;
  } else if (areYawAndPositionInited()) {  // Already initialized --> unary factor
    // Measurement
    graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "Lidar_unary_6D", int(realSenseOdometryRate), RealsenseOdometryTimeK, odomRealsensePtr->header.frame_id,
        dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->getRealsenseOdometryFrame(), lio_T_M_Lk, RealsenseOdomPoseUnaryNoise_);
    this->addUnaryPoseMeasurement(unary6DMeasurement);

    // Restart counter
    realsenseOdometryCallbackCounter_ = 0;
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(
      measRealsense_mapImuPathPtr_, odomRealsensePtr->header.frame_id, odomRealsensePtr->header.stamp,
      (lio_T_M_Lk * staticTransformsPtr_
                        ->rv_T_frame1_frame2(dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->getRealsenseOdometryFrame(),
                                             staticTransformsPtr_->getImuFrame())
                        .matrix())
          .block<3, 1>(0, 3),
      graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasMapRealsensePath_.publish(measRealsense_mapImuPathPtr_);
}


//---------------------------------------------------------------
void PositionGraphEstimator::publishState_(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Lookup I->B, also influenced by rotation of cabin
  static tf::StampedTransform transform_I_B;
  tfListener_.waitForTransform(staticTransformsPtr_->getImuFrame(),
                               dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                               ros::Duration(0.1));
  tfListener_.lookupTransform(staticTransformsPtr_->getImuFrame(),
                              dynamic_cast<PositionGraphStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                              transform_I_B);
  // Update Imu->Base transformation
  graph_msf::tfToIsometry3(transform_I_B, staticTransformsPtr_->lv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                                                                   staticTransformsPtr_->getBaseLinkFrame()));
  // Updaate Base->Imu transformation
  staticTransformsPtr_->lv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getBaseLinkFrame()).inverse();

  // Publish state
  graph_msf::GraphMsfRos::publishState_(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
}

}  // namespace positiongraph_se