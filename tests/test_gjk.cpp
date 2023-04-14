//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#define BOOST_TEST_MODULE COLBENCH_GJK

#include <iostream>
#include <boost/test/included/unit_test.hpp>
#include "colbench/fwd.hpp"
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include "hpp/fcl/shape/geometric_shapes.h"
#include "hpp/fcl/distance.h"

using pinocchio::SE3;
using pinocchio::toFclTransform3f;
using hpp::fcl::Ellipsoid;
using hpp::fcl::DistanceRequest;
using hpp::fcl::DistanceResult;
using hpp::fcl::Ellipsoid;
using Eigen::Vector3d;

BOOST_AUTO_TEST_CASE(colbench_test){
  Ellipsoid shape1(1., 1., 1.);
  SE3 M1 = SE3::Identity();
  Ellipsoid shape2(1., 1., 1.);
  SE3 M2 = SE3::Random();
  M2.translation(Vector3d(2.5, 0., 0.));

  DistanceRequest req;
  DistanceResult res;
  double dist = hpp::fcl::distance(&shape1, toFclTransform3f(M1), &shape2, toFclTransform3f(M2), req, res);

  BOOST_CHECK(std::abs(dist - 0.5) < 1e-8);
}
