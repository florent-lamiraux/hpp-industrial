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

#ifndef HPP_INDUSTRIAL_FWD_HH
#define HPP_INDUSTRIAL_FWD_HH

#include <hpp/manipulation/fwd.hh>

namespace hpp {
namespace industrial {
namespace pathPlanner {
HPP_PREDEF_CLASS(CartesianRRT);
typedef shared_ptr<CartesianRRT> CartesianRRTPtr_t;
}
typedef constraints::DifferentiableFunctionPtr_t DifferentiableFunctionPtr_t;
typedef constraints::ImplicitPtr_t ImplicitPtr_t;
typedef constraints::Implicit Implicit;
typedef constraints::TransformationR3xSO3 TransformationR3xSO3;
typedef constraints::Transform3s Transform3s;
typedef constraints::ComparisonTypes_t ComparisonTypes_t;

typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
typedef core::ConfigProjector ConfigProjector;
typedef core::ConnectedComponentPtr_t ConnectedComponentPtr_t;
typedef core::ConnectedComponents_t ConnectedComponents_t;
typedef core::ConstraintSetPtr_t ConstraintSetPtr_t;
typedef core::ConstraintSet ConstraintSet;
typedef core::DistancePtr_t DistancePtr_t;
typedef core::PathPtr_t PathPtr_t;
typedef core::PathProjectorPtr_t PathProjectorPtr_t;
typedef core::PathValidationPtr_t PathValidationPtr_t;
typedef core::PathValidationReportPtr_t PathValidationReportPtr_t;
typedef core::PathVectorPtr_t PathVectorPtr_t;
typedef core::PathVector PathVector;
typedef core::NearestNeighborPtr_t NearestNeighborPtr_t;
typedef core::NodeVector_t NodeVector_t;
typedef core::NodePtr_t NodePtr_t;
typedef core::Nodes_t Nodes_t;
typedef core::ParameterDescription ParameterDescription;
typedef core::Problem Problem;
typedef core::Parameter Parameter;
typedef core::SteeringMethodPtr_t SteeringMethodPtr_t;

typedef pinocchio::DevicePtr_t DevicePtr_t;
typedef pinocchio::Configuration_t Configuration_t;
typedef pinocchio::ConfigurationIn_t ConfigurationIn_t;
typedef pinocchio::JointPtr_t JointPtr_t;
typedef pinocchio::value_type value_type;
}
}
#endif // HPP_INDUSTRIAL_FWD_HH
