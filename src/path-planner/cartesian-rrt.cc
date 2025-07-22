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

#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/path-planning-failed.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validations.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/manipulation/path-planner/end-effector-trajectory.hh>
#include <hpp/manipulation/steering-method/end-effector-trajectory.hh>

#include <hpp/industrial/path-planner/cartesian-rrt.hh>

namespace hpp {
namespace industrial {
namespace pathPlanner {

CartesianRRTPtr_t CartesianRRT::create(const core::ProblemConstPtr_t& problem,
				       const JointPtr_t& joint) {
  CartesianRRT* ptr(new CartesianRRT(problem, joint));
  CartesianRRTPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

CartesianRRTPtr_t CartesianRRT::create(const core::ProblemConstPtr_t& problem,
    const core::RoadmapPtr_t& roadmap, const JointPtr_t& joint) {
  CartesianRRT* ptr(new CartesianRRT(problem, roadmap, joint));
  CartesianRRTPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

void CartesianRRT::init(CartesianRRTWkPtr_t weak) {
  core::PathPlanner::init(weak);
  weak_ = weak;
}

CartesianRRT::CartesianRRT(const core::ProblemConstPtr_t& problem, const JointPtr_t& joint) :
  core::PathPlanner(problem) {
  // Copy problem without path validation since EndEffectorTrajectory planner is used as
  // a steering method.
  pb_ = core::Problem::createCopy(problem);
  pb_->pathValidation(core::PathValidations::create());
  sm_ = manipulation::steeringMethod::EndEffectorTrajectory::create(pb_);
  pb_->steeringMethod(sm_);
  pp_ = manipulation::pathPlanner::EndEffectorTrajectory::create(pb_);
  pp_->maxIterations(1);
  pp_->nRandomConfig(0);
  // TODO: set through a parameter
  pp_->nDiscreteSteps(20);
  ImplicitPtr_t constraint(
      Implicit::create(TransformationR3xSO3::create("end-effector pose", problem->robot(),
						    joint, Transform3s::Identity()),
		       ComparisonTypes_t(6, constraints::Equality)));
  sm_->trajectoryConstraint(constraint);
  ConstraintSetPtr_t cs;
  ConfigProjectorPtr_t cp;
  if (!sm_->constraints()) {
    // Insert the trajectory constraint in the set of the steering method
    cs = ConstraintSet::create(problem->robot(), "CartesianRRT/sm/ConstraintSet");
    // TODO: set through parameters
    cp = ConfigProjector::create(problem->robot(),
				 "CartesianRRT/sm/ConfigProjector", 1e-5, 40);
    cs->addConstraint(cp);
    sm_->constraints(cs);
  } else {
    cs = sm_->constraints();
    cp = cs->configProjector();
  }
  cp->add(constraint, 0);
}

CartesianRRT::CartesianRRT(const core::ProblemConstPtr_t& problem,
                           const core::RoadmapPtr_t& roadmap, const JointPtr_t& joint) :
  core::PathPlanner(problem, roadmap) {
  // Copy problem without path validation since EndEffectorTrajectory planner is used as
  // a steering method.
  pb_ = core::Problem::createCopy(problem);
  pb_->pathValidation(core::PathValidations::create());
  sm_ = manipulation::steeringMethod::EndEffectorTrajectory::create(pb_);
  pb_->steeringMethod(sm_);
  pp_ = manipulation::pathPlanner::EndEffectorTrajectory::create(pb_);
  pp_->maxIterations(1);
  pp_->nRandomConfig(0);
  // TODO: set through a parameter
  pp_->nDiscreteSteps(20);
  ImplicitPtr_t constraint(
      Implicit::create(TransformationR3xSO3::create("end-effector pose", problem->robot(),
						    joint, Transform3s::Identity()),
		       ComparisonTypes_t(6, constraints::Equality)));
  sm_->trajectoryConstraint(constraint);
  ConstraintSetPtr_t cs;
  ConfigProjectorPtr_t cp;
  if (!sm_->constraints()) {
    // Insert the trajectory constraint in the set of the steering method
    cs = ConstraintSet::create(problem->robot(), "CartesianRRT/sm/ConstraintSet");
    // TODO: set through parameters
    cp = ConfigProjector::create(problem->robot(),
				 "CartesianRRT/sm/ConfigProjector", 1e-5, 40);
    cs->addConstraint(cp);
    sm_->constraints(cs);
  } else {
    cs = sm_->constraints();
    cp = cs->configProjector();
  }
  cp->add(constraint, 0);
}

bool belongs(ConfigurationIn_t q, const Nodes_t& nodes) {
  for (const auto& node : nodes) {
    if (node->configuration() == q) return true;
  }
  return false;
}

void CartesianRRT::oneStep() {
  value_type stepRatio =
      problem()
          ->getParameter("DiffusingPlanner/extensionStepRatio")
          .floatValue();

  typedef std::tuple<NodePtr_t, Configuration_t, PathPtr_t> DelayedEdge_t;
  typedef std::vector<DelayedEdge_t> DelayedEdges_t;
  DelayedEdges_t delayedEdges;
  DevicePtr_t robot(problem()->robot());
  PathValidationPtr_t pathValidation(problem()->pathValidation());
  Nodes_t newNodes, nearestNeighbors;
  PathPtr_t validPath, path;
  // Pick a random node
  Configuration_t q_rand;
  problem()->configurationShooter()->shoot(q_rand);
  //
  // First extend each connected component toward q_rand
  //
  for (const auto& cc : roadmap()->connectedComponents()) {
    // Find nearest node in roadmap
    value_type distance;
    NodePtr_t near = roadmap()->nearestNode(q_rand, cc, distance);
    nearestNeighbors.push_back(near);
    path = extend(near, q_rand);
    if (path) {
      PathValidationReportPtr_t report;
      bool pathValid = pathValidation->validate(path, false, validPath, report);
      // Insert new path to q_near in roadmap
      value_type t_final = validPath->timeRange().second;
      if (t_final != path->timeRange().first) {
        if (!pathValid && stepRatio > 0 && stepRatio < 1.) {
          value_type t0 = validPath->timeRange().first;
          validPath =
              validPath->extract(t0, t0 + validPath->length() * stepRatio);
        }
        Configuration_t q_new(validPath->end());
        if (!pathValid || !belongs(q_new, newNodes)) {
          newNodes.push_back(
              roadmap()->addNodeAndEdges(near, q_new, validPath));
        } else {
          // Store edges to add for later insertion.
          // Adding edges while looping on connected components is indeed
          // not recommended.
          delayedEdges.push_back(DelayedEdge_t(near, q_new, validPath));
        }
      }
    }
  }
  // Insert delayed edges
  for (const auto& edge : delayedEdges) {
    const NodePtr_t& near = std::get<0>(edge);
    Configuration_t q_new = std::get<1>(edge);
    const PathPtr_t& validPath = std::get<2>(edge);
    NodePtr_t newNode = roadmap()->addNode(q_new);
    roadmap()->addEdge(near, newNode, validPath);
    roadmap()->addEdge(newNode, near, validPath->reverse());
  }

  //
  // Second, try to connect new nodes with other connected components if close enough
  //
  const SteeringMethodPtr_t& sm(problem()->steeringMethod());
  for (const auto &node1 : newNodes) {
    /// Try connecting to the other new nodes.
    const ConnectedComponentPtr_t& cc1(node1->connectedComponent());
    for (const auto& cc2 : roadmap()->connectedComponents()) {
      if (cc1 == cc2) continue;
      // TODO: set through parameter
      value_type maxDistance = 1.;
      NodeVector_t nearestNodes(roadmap()->nearestNeighbor()->withinBall
				(node1->configuration(), cc2, maxDistance));
      for (const auto& node2 : nearestNodes) {
	Configuration_t q1(node1->configuration()), q2(node2->configuration());
	path = (*sm)(q1, q2);
	if (!path) continue;

	PathProjectorPtr_t pp = problem()->pathProjector();
	if (pp) {
	  PathPtr_t proj;
	  // If projection failed, continue
	  if (!pp->apply(path, proj)) continue;
	  path = proj;
	}
	PathValidationReportPtr_t report;
	bool valid = pathValidation->validate(path, false, validPath, report);
	if (valid) {
	  roadmap()->addEdge(node1, node2, path);
	  roadmap()->addEdge(node2, node1, path->reverse());
	}
      }
    }
  }
}

PathPtr_t CartesianRRT::extend(const NodePtr_t& near, ConfigurationIn_t target) {
  const SteeringMethodPtr_t& sm(problem()->steeringMethod());
  const ConstraintSetPtr_t& constraints(sm->constraints());
  Configuration_t qProj;
  if (constraints) {
    ConfigProjectorPtr_t configProjector(constraints->configProjector());
    if (configProjector) {
      configProjector->projectOnKernel(near->configuration(), target, qProj);
    } else {
      qProj = target;
    }
    if (!constraints->apply(qProj)) {
      return PathPtr_t();
    }
  } else {
    qProj = target;
  }
  assert(!qProj.hasNaN());
  // Here, qProj is a configuration that satisfies the constraints
  // or target if there are no constraints.
  pb_->initConfig(near->configuration());
  pb_->addGoalConfig(qProj);
  // Build linear trajectory of end-effector
  DifferentiableFunctionPtr_t f(sm_->trajectoryConstraint()->functionPtr());
  Eigen::Matrix<value_type, 7,2> m;
  m.col(0) = (*f)(near->configuration()).vector();
  m.col(1) = (*f)(qProj).vector();

  PathPtr_t path;
  try {
    PathPtr_t straightEe(
        manipulation::steeringMethod::EndEffectorTrajectory::makePiecewiseLinearTrajectory(
	    m, Eigen::Matrix<value_type, 6, 1>::Ones()));
    sm_->trajectory(straightEe, true);
    path = pp_->solve();
  } catch (const path_planning_failed& exc) {
    return PathPtr_t();
  }
  value_type stepLength =
      problem()
          ->getParameter("DiffusingPlanner/extensionStepLength")
          .floatValue();
  if (stepLength > 0 && path->length() > stepLength) {
    value_type t0 = path->timeRange().first;
    path = path->extract(t0, t0 + stepLength);
  }
  PathProjectorPtr_t pp = problem()->pathProjector();
  if (pp) {
    PathPtr_t proj;
    pp->apply(path, proj);
    return proj;
  }
  return path;
}

}
}
}
