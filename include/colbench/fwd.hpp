//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#ifndef COLBENCH_FWD_HPP
#define COLBENCH_FWD_HPP

#include "hpp/fcl/mesh_loader/loader.h"
#include "hpp/fcl/BVH/BVH_model.h"
#include "hpp/fcl/distance.h"
#include "hpp/fcl/shape/convex.h"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include "pinocchio/serialization/se3.hpp"
#include "pinocchio/serialization/geometry.hpp"
#include <Eigen/Core>

using pinocchio::SE3;
using pinocchio::toFclTransform3f;
using hpp::fcl::Transform3f;
using hpp::fcl::BVHModelPtr_t;
using hpp::fcl::BVHModel;
using hpp::fcl::MeshLoader;
using hpp::fcl::NODE_TYPE;
using hpp::fcl::BV_AABB;
using hpp::fcl::AABB;
using hpp::fcl::ConvexBase;
using hpp::fcl::Convex;
using hpp::fcl::Triangle;
using hpp::fcl::ShapeBase;
using hpp::fcl::details::GJK;
using hpp::fcl::details::MinkowskiDiff;
using hpp::fcl::support_func_guess_t;
using hpp::fcl::DistanceRequest;
using hpp::fcl::DistanceResult;
using hpp::fcl::Timer;
using Eigen::Vector3d;
using Eigen::Matrix3d;


#endif
