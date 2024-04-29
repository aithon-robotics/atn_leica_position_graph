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

namespace excavator_se {

ExcavatorEstimator::ExcavatorEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {

  std::cout << YELLOW_START << "LeicaPositionEstimator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static Transforms
  staticTransformsPtr_ = std::make_shared<ExcavatorStaticTransforms>(privateNodePtr);

  // GNSS Handler
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  // Setup
  if (not ExcavatorEstimator::setup()) {
    REGULAR_COUT << RED_START << " Failed to set up." << COLOR_END << std::endl;
    throw std::runtime_error("LeicaPositionEstimator could not be initialized");
  }

  std::cout << YELLOW_START << "LeicaPositionEstimator" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}


//---------------------------------------------------------------
bool ExcavatorEstimator::setup() {

  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  ExcavatorEstimator::readParams_(privateNode_);

  // Super class
  if (not graph_msf::GraphMsfRos::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Publishers ----------------------------
  ExcavatorEstimator::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  ExcavatorEstimator::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  ExcavatorEstimator::initializeMessages_(privateNode_);

  // Static Transforms
  staticTransformsPtr_->findTransformations();

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}


//---------------------------------------------------------------
void ExcavatorEstimator::initializePublishers_(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Publishers..." << COLOR_END << std::endl;

  // Paths
  pubMeasWorldPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measGnssL_path_world_gnssL", ROS_QUEUE_SIZE);
}

void ExcavatorEstimator::initializeSubscribers_(ros::NodeHandle& privateNode) {

  subPosition_ = privateNode.subscribe<geometry_msgs::PointStamped>(
    "/gnss_topic_1", ROS_QUEUE_SIZE,  &ExcavatorEstimator::positionCallback_, this, ros::TransportHints().tcpNoDelay());

  std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Position subscriber (on Gnss_topic_1)." << std::endl;
  return;
}


//---------------------------------------------------------------
void ExcavatorEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Messages..." << COLOR_END << std::endl;

  // Paths
  measGnss_worldPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}


//---------------------------------------------------------------
void ExcavatorEstimator::positionCallback_(const geometry_msgs::PointStamped::ConstPtr& LeicaPositionPtr) {
  
  // Static variables
  static Eigen::Vector3d zeroCoord(0.0, 0.0, 0.0);
  static bool PositionHealthyFlag__ = true;
  static int positionCallbackCounter__ = 0;

  // Translate to Eigen
  Eigen::Vector3d positionCoord = Eigen::Vector3d(LeicaPositionPtr->point.x, LeicaPositionPtr->point.y, LeicaPositionPtr->point.z);
  Eigen::Vector3d positionCovarianceXYZ(0.001, 0.001, 0.001); // TODO: Set proper values

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
                                 dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getCabinFrame(),
                                 dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLeftGnssFrame())) {
      REGULAR_COUT << " Set yaw and position successfully." << std::endl;
    } else {
      REGULAR_COUT << " Could not set yaw and position." << std::endl;
      }
  } else {  
    // Already initialized --> add position measurement to graph
    positionCovarianceXYZ = Eigen::Vector3d(std::max(positionCovarianceXYZ(0), gnssPositionUnaryNoise_),
                                            std::max(positionCovarianceXYZ(1), gnssPositionUnaryNoise_),
                                            std::max(positionCovarianceXYZ(2), gnssPositionUnaryNoise_));
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> meas_W_t_W_Position(
      "Leica-Position", int(gnssRate_), ros::Time::now().toSec(), staticTransformsPtr_->getWorldFrame() + "_ENU",
      dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLeftGnssFrame(), positionCoord, positionCovarianceXYZ,
      GNSS_POSITION_COVARIANCE_VIOLATION_THRESHOLD);
    if (!this->addPositionMeasurement(meas_W_t_W_Position)) {
      if (PositionHealthyFlag__) {
        REGULAR_COUT << RED_START << " Gnss left position measurement not added." << COLOR_END << std::endl;
        PositionHealthyFlag__ = false;
      }
    } else if (!PositionHealthyFlag__) {
      REGULAR_COUT << GREEN_START << " Gnss left position measurement returned." << COLOR_END << std::endl;
      PositionHealthyFlag__ = true;
    }
  }

  // Visualizations
  addToPathMsg(measGnss_worldPositionPathPtr_, staticTransformsPtr_->getWorldFrame(), LeicaPositionPtr->header.stamp, positionCoord,
               graphConfigPtr_->imuBufferLength * 4);
  pubMeasWorldPositionPath_.publish(measGnss_worldPositionPathPtr_);
}


//---------------------------------------------------------------
void ExcavatorEstimator::publishState_(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Lookup I->B, also influenced by rotation of cabin
  static tf::StampedTransform transform_I_B;
  tfListener_.waitForTransform(staticTransformsPtr_->getImuFrame(),
                               dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                               ros::Duration(0.1));
  tfListener_.lookupTransform(staticTransformsPtr_->getImuFrame(),
                              dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
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

}  // namespace excavator_se