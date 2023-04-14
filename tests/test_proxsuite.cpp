//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#define BOOST_TEST_MODULE COLBENCH_PROXSUITE

#include <iostream>
#include <boost/test/included/unit_test.hpp>
#include "colbench/utility.hpp"
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include "colbench/serialization.hpp"
#include "benchmarks/config.h"

BOOST_AUTO_TEST_CASE(test_proxsuite_vs_gjk){
  // Load meshes from ycb dataset -- needs to be downloaded first
  std::filesystem::path path1(std::string(COLBENCH_YCB_DIR) + "/data/ycb_data/002_master_chef_can/google_16k/nontextured.stl");
  std::filesystem::path path2(std::string(COLBENCH_YCB_DIR) + "/data/ycb_data/003_cracker_box/google_16k/nontextured.stl");
  bool exists = std::filesystem::exists(path1);
  BOOST_CHECK(exists);
  exists = std::filesystem::exists(path2);
  BOOST_CHECK(exists);

  // Load meshes
  std::shared_ptr<ConvexBase> shape1 = colbench::loadConvexMesh(path1);
  std::cout << "Num vertices shape1: " << shape1->num_points << std::endl;
  std::shared_ptr<ConvexBase> shape2 = colbench::loadConvexMesh(path2);
  std::cout << "Num vertices shape2: " << shape2->num_points << std::endl;

  // Random config
  SE3 M1 = SE3::Identity(); Transform3f T1 = pinocchio::toFclTransform3f(M1);
  SE3 M2 = SE3::Random(); Transform3f T2 = pinocchio::toFclTransform3f(M2);

  DistanceRequest req;
  DistanceResult res;

  // -------------------------
  // -------- HPP-FCL --------
  // -------------------------
  double desired_dist = 1e-3;
  colbench::bringShapesToDist(shape1.get(), T1, shape2.get(), T2, desired_dist);
  double dist_hppfcl = hpp::fcl::distance(shape1.get(), T1, shape2.get(), T2, req, res);
  std::cout << "Dist HPPFCL: " << dist_hppfcl << std::endl;
  BOOST_CHECK(std::abs(dist_hppfcl - desired_dist) < 1e-6);

  // -------------------------
  // -------- PROX-QP --------
  // -------------------------
  // create qp object and pass some settings
  Convex<Triangle>* cvx1 = static_cast<Convex<Triangle>*>(shape1.get());
  cvx1->buildDoubleDescription();
  Convex<Triangle>* cvx2 = static_cast<Convex<Triangle>*>(shape2.get());
  cvx2->buildDoubleDescription();
  proxsuite::proxqp::dense::QP<double> qp = colbench::setupProxQPSolver(cvx1, T1, cvx2, T2);
  double eps_abs = 1e-2;
  qp.settings.eps_abs = eps_abs;
  qp.settings.compute_timings = true;
  qp.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
  qp.solve();

  // BOOST_CHECK((res.nearest_points[0] - qp.results.x.block<3, 1>(0, 0)).norm() < 1e-4);
  // BOOST_CHECK((res.nearest_points[1] - qp.results.x.block<3, 1>(3, 0)).norm() < 1e-4);
  double dist_proxqp = (qp.results.x.block<3, 1>(0, 0) - qp.results.x.block<3, 1>(3, 0)).norm();
  std::cout << "Dist ProxQP: " << dist_proxqp << std::endl;
  BOOST_CHECK(std::abs(dist_proxqp - dist_hppfcl) < 1e-4);
  std::cout << "ProxQP timings: " << qp.results.info.run_time << std::endl;
}

BOOST_AUTO_TEST_CASE(test_proxsuite_vs_gjk_col_problem){
  // Load the shapes
  std::vector<BVHModel<AABB>> bvhs;
  std::string shapes_path(std::string(COLBENCH_YCB_DIR) + "/data/ycb_cvx_hulls.bin");
  bool exists = std::filesystem::exists(shapes_path);
  BOOST_CHECK(exists);
  colbench::loadSerialize(bvhs, shapes_path);
  for (size_t i = 0; i < bvhs.size(); i++) {
    bvhs[i].buildConvexRepresentation(false);
    bvhs[i].convex->computeLocalAABB(); // Used for solver initialization
      Convex<Triangle>* shape;
      shape = static_cast<Convex<Triangle>*>(bvhs[i].convex.get());
      shape->buildDoubleDescription();
  }

  // Load the collision problems
  std::vector<CollisionProblem> problems;
  std::string problems_path(std::string(COLBENCH_YCB_DIR) + "/data/ycb_collision_problems.bin");
  exists = std::filesystem::exists(problems_path);
  BOOST_CHECK(exists);
  colbench::loadSerialize(problems, problems_path);

  // Load problem
  Transform3f T1, T2;
  Convex<Triangle>* shape1; Convex<Triangle>* shape2;
  CollisionProblem& problem = problems[0];

  shape1 = static_cast<Convex<Triangle>*>(bvhs[problem.id_shape1].convex.get());
  std::cout << "Num vertices shape1: " << shape1->num_points << std::endl;
  shape2 = static_cast<Convex<Triangle>*>(bvhs[problem.id_shape2].convex.get());
  std::cout << "Num vertices shape2: " << shape2->num_points << std::endl;

  // Scale the problem according to user's distance input
  double desired_dist = 1e-2;
  problem.scaleToDist(desired_dist);

  T1 = toFclTransform3f(problem.M1);
  T2 = toFclTransform3f(problem.M2);


  DistanceRequest req;
  DistanceResult res;

  // -------------------------
  // -------- HPP-FCL --------
  // -------------------------
  double dist = hpp::fcl::distance(shape1, T1, shape2, T2, req, res);
  BOOST_CHECK(std::abs(dist - desired_dist) < 1e-6);

  // -------------------------
  // -------- PROX-QP --------
  // -------------------------
  // create qp object and pass some settings
  proxsuite::proxqp::dense::QP<double> qp = colbench::setupProxQPSolver(shape1, T1, shape2, T2);
  double eps_abs = 1e-8;
  qp.settings.eps_abs = eps_abs;
  qp.settings.compute_timings = true;
  qp.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
  qp.solve();

  BOOST_CHECK((res.nearest_points[0] - qp.results.x.block<3, 1>(0, 0)).norm() < 1e-6);
  BOOST_CHECK((res.nearest_points[1] - qp.results.x.block<3, 1>(3, 0)).norm() < 1e-6);
  dist = (qp.results.x.block<3, 1>(0, 0) - qp.results.x.block<3, 1>(3, 0)).norm();
  BOOST_CHECK(std::abs(dist - desired_dist) < 1e-6);
  std::cout << "ProxQP timings: " << qp.results.info.run_time << std::endl;
}

BOOST_AUTO_TEST_CASE(test_proxsuite_cube1){
  // Load problem
  Transform3f T1, T2;
  T1 = Transform3f::Identity();
  SE3 M2 = SE3::Random();
  T2 = toFclTransform3f(M2);
  Convex<Triangle>* shape1; Convex<Triangle>* shape2;

  shape1 = colbench::create_cube(0.3, 0.3, 0.3);
  shape1->buildDoubleDescription();
  std::cout << "Num vertices shape1: " << shape1->num_points << std::endl;
  std::cout << "Num normals shape1: " << shape1->num_normals << std::endl;
  shape2 = colbench::create_cube(0.3, 0.3, 0.3);
  shape2->buildDoubleDescription();
  std::cout << "Num vertices shape2: " << shape2->num_points << std::endl;
  std::cout << "Num normals shape2: " << shape2->num_normals << std::endl;

  DistanceRequest req;
  DistanceResult res;

  // -------------------------
  // -------- HPP-FCL --------
  // -------------------------
  double dist_hppfcl = hpp::fcl::distance(shape1, T1, shape2, T2, req, res);
  std::cout << "Dist hppfcl: " << dist_hppfcl << std::endl;

  // -------------------------
  // -------- PROX-QP --------
  // -------------------------
  // create qp object and pass some settings
  proxsuite::proxqp::dense::QP<double> qp = colbench::setupProxQPSolver(shape1, T1, shape2, T2);
  double eps_abs = 1e-8;
  qp.settings.eps_abs = eps_abs;
  qp.settings.compute_timings = true;
  qp.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
  qp.solve();

  BOOST_CHECK((res.nearest_points[0] - qp.results.x.block<3, 1>(0, 0)).norm() < 1e-6);
  BOOST_CHECK((res.nearest_points[1] - qp.results.x.block<3, 1>(3, 0)).norm() < 1e-6);
  double dist_proxqp = (qp.results.x.block<3, 1>(0, 0) - qp.results.x.block<3, 1>(3, 0)).norm();
  std::cout << "Dist proxqp: " << dist_proxqp << std::endl;
  BOOST_CHECK(std::abs(dist_proxqp - dist_hppfcl) < 1e-6);
  std::cout << "ProxQP timings: " << qp.results.info.run_time << std::endl;
}

BOOST_AUTO_TEST_CASE(test_proxsuite_cube2){
  // Load problem
  Transform3f T1, T2;
  T1 = Transform3f::Identity();
  SE3 M2 = SE3::Random();
  T2 = toFclTransform3f(M2);
  Convex<Triangle>* shape1; Convex<Triangle>* shape2;

  shape1 = colbench::create_cube_proxqp(0.3, 0.3, 0.3);
  std::cout << "Num vertices shape1: " << shape1->num_points << std::endl;
  std::cout << "Num normals shape1: " << shape1->num_normals << std::endl;
  shape2 = colbench::create_cube_proxqp(0.3, 0.3, 0.3);
  std::cout << "Num vertices shape2: " << shape2->num_points << std::endl;
  std::cout << "Num normals shape2: " << shape2->num_normals << std::endl;

  DistanceRequest req;
  DistanceResult res;

  // -------------------------
  // -------- HPP-FCL --------
  // -------------------------
  double dist_hppfcl = hpp::fcl::distance(shape1, T1, shape2, T2, req, res);
  std::cout << "Dist hppfcl: " << dist_hppfcl << std::endl;

  // -------------------------
  // -------- PROX-QP --------
  // -------------------------
  // create qp object and pass some settings
  proxsuite::proxqp::dense::QP<double> qp = colbench::setupProxQPSolver(shape1, T1, shape2, T2);
  double eps_abs = 1e-8;
  qp.settings.eps_abs = eps_abs;
  qp.settings.compute_timings = true;
  qp.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
  qp.solve();

  BOOST_CHECK((res.nearest_points[0] - qp.results.x.block<3, 1>(0, 0)).norm() < 1e-6);
  BOOST_CHECK((res.nearest_points[1] - qp.results.x.block<3, 1>(3, 0)).norm() < 1e-6);
  double dist_proxqp = (qp.results.x.block<3, 1>(0, 0) - qp.results.x.block<3, 1>(3, 0)).norm();
  std::cout << "Dist proxqp: " << dist_proxqp << std::endl;
  BOOST_CHECK(std::abs(dist_proxqp - dist_hppfcl) < 1e-6);
  std::cout << "ProxQP timings: " << qp.results.info.run_time << std::endl;
}
