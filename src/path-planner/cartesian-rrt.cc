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
#include <hpp/core/distance.hh>
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
    if (!cp) {
      // TODO: set through parameters
      cp = ConfigProjector::create(problem->robot(),
				   "CartesianRRT/sm/ConfigProjector", 1e-5, 40);
      cs->addConstraint(cp);
    }
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
  extendMaxLength_ =
      problem()->getParameter("CartesianRRT/maxStepLength").floatValue();
  if (extendMaxLength_ <= 0)
    extendMaxLength_ = std::sqrt(problem()->robot()->numberDof());
  gamma_ = problem()->getParameter("CartesianRRT/gamma").floatValue();

  typedef std::tuple<NodePtr_t, Configuration_t, PathPtr_t> DelayedEdge_t;
  typedef std::vector<DelayedEdge_t> DelayedEdges_t;
  DelayedEdges_t delayedEdges;
  DevicePtr_t robot(problem()->robot());
  PathValidationPtr_t pathValidation(problem()->pathValidation());
  Nodes_t newNodes, nearestNeighbors;
  PathPtr_t validPath, path;
  // Pick a random node
  Configuration_t q_rand(problem()->robot()->configSize());
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
      Configuration_t q_new = path->end();
      // Insert new path to q_near in roadmap
      if (pathValid) {
        if (!belongs(q_new, newNodes)) {
          newNodes.push_back(
              roadmap()->addNodeAndEdges(near, q_new, validPath));
        }
	// Store edges to add for later insertion.
	// Adding edges while looping on connected components is indeed
	// not recommended.
	delayedEdges.push_back(DelayedEdge_t(near, q_new, validPath));
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
      value_type maxDistance =
      problem()->getParameter("CartesianRRT/maxDistanceInterpolation").floatValue();
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

PathPtr_t CartesianRRT::cartesianSpaceInterpolation(ConfigurationIn_t q1, ConfigurationIn_t q2) {
  value_type maxDistance =
    problem()->getParameter("CartesianRRT/maxDistanceInterpolation").floatValue();
  pb_->initConfig(q1);
  pb_->addGoalConfig(q2);
  // Build linear trajectory of end-effector
  DifferentiableFunctionPtr_t f(sm_->trajectoryConstraint()->functionPtr());
  Eigen::Matrix<value_type, 2,7> m;
  m.row(0) = (*f)(q1).vector();
  m.row(1) = (*f)(q2).vector();
  PathPtr_t path;
  try {
    PathPtr_t straightEe(
        manipulation::steeringMethod::EndEffectorTrajectory::makePiecewiseLinearTrajectory(
	    m, Eigen::Matrix<value_type, 6, 1>::Ones()));
    sm_->trajectory(straightEe, true);
    path = pp_->solve();
  } catch (const core::path_planning_failed& exc) {
    return PathPtr_t();
  }
  // Make sure path is a PathVector to be able to append another path to it.
  PathVectorPtr_t pv;
  pv = HPP_DYNAMIC_PTR_CAST(PathVector, path);
  if (!pv) {
    pv = PathVector::create(path->outputSize(), path->outputDerivativeSize());
    pv->appendPath(path);
  }
  // Due to projection inaccuracy, the Cartesian path might not end up exactly in q2 even
  // in case of success.
  // If end of path is close enough to q2, we add a straight path between those.
  core::DistancePtr_t distance(problem()->distance());
  SteeringMethodPtr_t sm(problem()->steeringMethod());
  if (((*distance)(path->end(), q2) <= maxDistance) && (path->end() != q2)) {
    PathPtr_t p((*sm)(path->end(), q2));
    if (p) {
      pv->appendPath(p);
    }
  }
  return pv;
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
  PathPtr_t path(cartesianSpaceInterpolation(near->configuration(), qProj));
  if (path && extendMaxLength_ > 0 && path->length() > extendMaxLength_) {
    value_type t0 = path->timeRange().first;
    path = path->extract(t0, t0 + extendMaxLength_);
  }
  PathProjectorPtr_t pp = problem()->pathProjector();
  if (pp) {
    PathPtr_t proj;
    pp->apply(path, proj);
    return proj;
  }
  return path;
}

void CartesianRRT::tryConnectInitAndGoals() {
  // TODO: set through parameter
  value_type maxDistance =
    problem()->getParameter("CartesianRRT/maxDistanceInterpolation").floatValue();
  // call steering method here to build a direct connexion
  const SteeringMethodPtr_t& sm(problem()->steeringMethod());
  PathValidationPtr_t pathValidation(problem()->pathValidation());
  PathProjectorPtr_t pathProjector(problem()->pathProjector());
  PathPtr_t validPath, projPath, path;
  NodePtr_t initNode = roadmap()->initNode();
  NearestNeighborPtr_t nn(roadmap()->nearestNeighbor());
  // Register edges to add to roadmap and add them after iterating
  // among the connected components.
  typedef std::tuple<NodePtr_t, NodePtr_t, PathPtr_t> FutureEdge_t;
  typedef std::vector<FutureEdge_t> FutureEdges_t;
  FutureEdges_t futureEdges;
  ConnectedComponentPtr_t initCC(initNode->connectedComponent());
  for (ConnectedComponents_t::iterator itCC(
           roadmap()->connectedComponents().begin());
       itCC != roadmap()->connectedComponents().end(); ++itCC) {
    if (*itCC != initCC) {
      value_type d;
      NodePtr_t near(nn->search(initNode->configuration(), *itCC, d, true));
      assert(near);
      path.reset();
      // If q1 and q2 are close enough, connect them with default steering method
      // Usually joint space interpolation
      Configuration_t q1(initNode->configuration());
      Configuration_t q2(near->configuration());
      if (d < maxDistance) {
	path = (*sm)(q1, q2);
      }
      // Otherwise, connect them with cartesian space interpolation
      else {
	path = cartesianSpaceInterpolation(q1, q2);
      }
      if (!path) continue;
      if (pathProjector) {
	if (!pathProjector->apply(path, projPath)) continue;
      } else {
	projPath = path;
      }
      if (projPath) {
	PathValidationReportPtr_t report;
	bool pathValid =
	  pathValidation->validate(projPath, false, validPath, report);
	if (pathValid && validPath->length() > 0) {
	  futureEdges.push_back(FutureEdge_t(initNode, near, projPath));
	}
      }
    }
  }
  for (NodeVector_t::const_iterator itn = roadmap()->goalNodes().begin();
       itn != roadmap()->goalNodes().end(); ++itn) {
    ConnectedComponentPtr_t goalCC((*itn)->connectedComponent());
    for (ConnectedComponents_t::iterator itCC(
             roadmap()->connectedComponents().begin());
         itCC != roadmap()->connectedComponents().end(); ++itCC) {
      if (*itCC != goalCC) {
        value_type d;
        NodePtr_t near(nn->search((*itn)->configuration(), *itCC, d, false));
        assert(near);
	path.reset();
	// If q1 and q2 are close enough, connect them with default steering method
	// Usually joint space interpolation
	Configuration_t q1(near->configuration());
	Configuration_t q2((*itn)->configuration());
	if (d < maxDistance) {
	  path = (*sm)(q1, q2);
	}
	// Otherwise, connect them with cartesian space interpolation
	else {
	  path = cartesianSpaceInterpolation(q1, q2);
	}
        if (!path) continue;
        if (pathProjector) {
          if (!pathProjector->apply(path, projPath)) continue;
        } else {
          projPath = path;
        }
        if (projPath) {
          PathValidationReportPtr_t report;
          bool pathValid =
              pathValidation->validate(projPath, false, validPath, report);
          if (pathValid && validPath->length() > 0) {
            futureEdges.push_back(FutureEdge_t(near, (*itn), projPath));
          }
        }
      }
    }
  }
  // Add edges
  for (const auto& e : futureEdges)
    roadmap()->addEdge(std::get<0>(e), std::get<1>(e), std::get<2>(e));
}

HPP_START_PARAMETER_DECLARATION(CartesianRRT)
Problem::declareParameter(ParameterDescription(Parameter::FLOAT,
    "CartesianRRT/maxDistanceInterpolation",
    "Maximal distance between two configurations to allow joint space interpolation",
    Parameter(.5)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "CartesianRRT/maxStepLength",
    "The maximum step length when extending. If negative, uses sqrt(dimension)",
    Parameter(-1.)));
Problem::declareParameter(ParameterDescription(Parameter::FLOAT, "CartesianRRT/gamma",
    "Parameter defined in Algorithm 5 of \"Sampling-based algorithms for optimal motion planning\""
    "by Karaman and Frazolli published in IJRR 2011", Parameter(1.)));
HPP_END_PARAMETER_DECLARATION(CartesianRRT);
}
}
}
