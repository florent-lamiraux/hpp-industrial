//
// Copyright (c) 2025 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_INDUSTRIAL_PATH_PLANNER_CARTESIAN_RRT_HH
#define HPP_INDUSTRIAL_PATH_PLANNER_CARTESIAN_RRT_HH

#include <hpp/industrial/fwd.hh>
#include <hpp/core/path-planner.hh>

namespace hpp {
namespace industrial {
namespace pathPlanner {
class CartesianRRT : public core::PathPlanner
{
public:
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  /// \param joint joint that should move along straight paths
  static CartesianRRTPtr_t create(const core::ProblemConstPtr_t& problem, const JointPtr_t& joint);
  /// Return shared pointer to new instance
  /// \param problem the path planning problem
  /// \param roadmap previously built roadmap
  static CartesianRRTPtr_t create(const core::ProblemConstPtr_t& problem,
      const core::RoadmapPtr_t& roadmap, const JointPtr_t& joint);
  /// One iteration of the planner
  virtual void oneStep();
  /// Extend a configuration toward a random configuration using Cartesian interpolation
  PathPtr_t extend(const NodePtr_t& near, ConfigurationIn_t target);
  /// Try to connect initial and goal configurations to existing roadmap
  virtual void tryConnectInitAndGoals();
  // Compute cartesian space interpolation between q1 and q2.
  // Note that in case of success, q2 might not be reached. Instead another configuration
  // with the same end-effector pose might be reached.
  PathPtr_t cartesianSpaceInterpolation(ConfigurationIn_t q1, ConfigurationIn_t q2);
protected:
  // Protected constructor
  CartesianRRT(const core::ProblemConstPtr_t& problem, const JointPtr_t& joint);
  // Protected constructor
  CartesianRRT(const core::ProblemConstPtr_t& problem, const core::RoadmapPtr_t& roadmap,
	       const JointPtr_t& joint);

private:
  /// Store weak pointer to itself
  void init(CartesianRRTWkPtr_t weak);
  JointPtr_t endEffector_;
  /// Steering method for EndEffectorTrajectory path planner
  manipulation::steeringMethod::EndEffectorTrajectoryPtr_t sm_;
  /// EndEffectorTrajectory path planner used as a steering method
  manipulation::pathPlanner::EndEffectorTrajectoryPtr_t pp_;
  /// Problem for the EndEffectorTrajectory path planner without path validation
  core::ProblemPtr_t pb_;
  value_type gamma_;
  /// Maximal path length with using function \ref extend.
  value_type extendMaxLength_;
  CartesianRRTWkPtr_t weak_;
}; // class CartesianRRT
}
}
}
#endif // HPP_INDUSTRIAL_PATH_PLANNER_CARTESIAN_RRT_HH
