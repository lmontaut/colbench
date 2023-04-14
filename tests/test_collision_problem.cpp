//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#define BOOST_TEST_MODULE COLLISION_PROBLEM 

#include <iostream>
#include <boost/test/included/unit_test.hpp>
#include <boost/archive/tmpdir.hpp>
#include "benchmarks/config.h"
#include "colbench/collision_problem.hpp"
#include "colbench/utility.hpp"
#include "colbench/serialization.hpp"

BOOST_AUTO_TEST_CASE(test_collision_problem_distance_Scale){
  // Generate a collision problem
  // -- Shapes
  std::filesystem::path shape_path1(std::string(COLBENCH_YCB_DIR) +
                                    "/data/ycb_data/002_master_chef_can/google_16k/nontextured.stl");
  bool exists1 = std::filesystem::exists(shape_path1);
  std::filesystem::path shape_path2(std::string(COLBENCH_YCB_DIR) +
                                    "/data/ycb_data/002_master_chef_can/google_16k/nontextured.stl");
  bool exists2 = std::filesystem::exists(shape_path2);
  std::string message_no_ycb = " does not exist.\nMake sure you have run `python benchmarks/ycb/ycb_download.py` and `python benchmarks/ycb/ycb_to_csv.py` in the root repository directory.";
  if(!exists1){
    std::cout << std::string(shape_path1) + message_no_ycb << std::endl;
  } else if (!exists2)  {
    std::cout << std::string(shape_path2) + message_no_ycb << std::endl;
  } else {
    std::shared_ptr<ConvexBase> shape1, shape2;
    shape1 = colbench::loadConvexMesh(shape_path1);
    shape2 = colbench::loadConvexMesh(shape_path2);

    // -- Problem
    SE3 M1 = SE3::Identity(); SE3 M2 = SE3::Random();
    DistanceRequest req; DistanceResult res;
    hpp::fcl::ComputeDistance distancer(shape1.get(), shape2.get());
    double dist = distancer(toFclTransform3f(M1), toFclTransform3f(M2), req, res);
    colbench::CollisionProblem problem_;
    problem_.M1 = M1;
    problem_.M2 = M2;
    problem_.unscaled_translation = M2.translation();
    problem_.unscaled_separation_vector = res.nearest_points[0] - res.nearest_points[1];
    problem_.normalized_separation_vector = problem_.unscaled_separation_vector.normalized();
    problem_.unscaled_dist = dist;
    M2.setRandom();

    // Save/Load it
    const std::string tmp_dir(boost::archive::tmpdir());
    const std::string filename = tmp_dir + "file.bin";
    colbench::storeSerialize(problem_, filename);

    colbench::CollisionProblem problem;
    colbench::loadSerialize(problem, filename);

    // Check that scaling the collision problem gives the good distance
    double desired_dist = 1e-2;
    problem.scaleToDist(desired_dist);
    res.clear();
    dist = distancer(toFclTransform3f(problem.M1), toFclTransform3f(problem.M2), req, res);
    BOOST_CHECK(std::abs(dist - desired_dist) < 1e-4);

    desired_dist = 1e-3;
    problem.scaleToDist(desired_dist);
    res.clear();
    dist = distancer(toFclTransform3f(problem.M1), toFclTransform3f(problem.M2), req, res);
    BOOST_CHECK(std::abs(dist - desired_dist) < 1e-4);

    desired_dist = -1e-2;
    problem.scaleToDist(desired_dist);
    res.clear();
    dist = distancer(toFclTransform3f(problem.M1), toFclTransform3f(problem.M2), req, res);
    BOOST_CHECK(std::abs(dist - desired_dist) < 1e-4);
  }
}
