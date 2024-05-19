/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef PositionGraphStaticTransforms_H
#define PositionGraphStaticTransforms_H
// Workspace
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

namespace positiongraph_se {

class PositionGraphStaticTransforms : public graph_msf::StaticTransformsTf {
 public:
  PositionGraphStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr,
                            const graph_msf::StaticTransforms& staticTransforms = graph_msf::StaticTransforms());

  // Setters ---------------------------------------------------------------
  void setPositionMeasFrame(const std::string& s) { positionMeasFrame_ = s; }
  void setBodyFrame(const std::string& s) { bodyFrame_ = s; }

  void setRealsenseOdometryFrame(const std::string& s) { realsenseFrame_ = s; }


  // Getters ---------------------------------------------------------------
  const std::string& getPositionMeasFrame() { return positionMeasFrame_; }
  const std::string& getBodyFrame() { return bodyFrame_; }
  const std::string& getRealsenseOdometryFrame() { return realsenseFrame_; }

 protected:  // Methods
  void findTransformations() override;

 private:  // Members
  // Robot frame names
  std::string positionMeasFrame_;
  std::string bodyFrame_;
  std::string realsenseFrame_;
};
}  // namespace positiongraph_se
#endif  // end AsopStaticTransforms_H
