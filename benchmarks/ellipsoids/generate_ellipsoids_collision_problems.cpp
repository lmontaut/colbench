//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#include <iostream>
#include "colbench/utility.hpp"
#include "benchmarks/config.h"
#include "colbench/csv.hpp"
#include "colbench/benchmark_setup.hpp"
#include "colbench/collision_problem.hpp"
#include "colbench/serialization.hpp"
#include "colbench/fwd.hpp"
#include <filesystem>

using hpp::fcl::Ellipsoid;

int main (int argc, char *argv[])
{
  po::options_description desc = colbench::createExecutableOptions(true);
  po::variables_map vm = colbench::parseExecutableArguments(argc, argv, desc);

  if (!colbench::showParserHelp(desc, vm)){
    // Create ellipsoids
    std::cout << "Creating ellipsoids" << std::endl;
    std::vector<Ellipsoid> ellipsoids;
    size_t num_ellipsoids;
    if (vm.count("num_ellipsoids")) {
      num_ellipsoids = vm["num_ellipsoids"].as<std::size_t>();
    } else {
      num_ellipsoids = 100;
      std::cout << "Selecting default value for `--num_ellipsoids`." << std::endl;
    }
    // Random stuff
    std::mt19937 gen(0); // random seed set to 0
    std::normal_distribution<double> distribution(0.1, 0.025);
    for (size_t i = 0; i < num_ellipsoids; i++) {
      Vector3d radii = Eigen::Vector3d::Zero().unaryExpr([&](float dummy){ return distribution(gen);});
      Ellipsoid ellipsoid(radii);
      ellipsoids.push_back(ellipsoid);
    }

    // Generate collision problems
    std::vector<CollisionProblem> problems;
    const SE3 M1 = SE3::Identity(); SE3 M2;
    DistanceRequest req;
    DistanceResult res;

    size_t num_problems_per_pair;
    if (vm.count("nprob_per_pair")) {
      num_problems_per_pair = vm["nprob_per_pair"].as<std::size_t>();
    } else {
      num_problems_per_pair = 100;
      std::cout << "Selecting default value for `--nprob_per_pair`." << std::endl;
    }
    std::cout << "Number of ellipsoids: " << num_ellipsoids << std::endl;
    std::cout << "Number of collision pairs: " << int(num_ellipsoids * (num_ellipsoids - 1) / 2) << std::endl;
    std::cout << "Number of problems per collision pair: " << num_problems_per_pair << std::endl;
    std::cout << "Generating collision problems..." << std::endl;

    size_t pair_id = 0;
    for (size_t i = 0; i < num_ellipsoids; i++) {
      for (size_t j = i; j < num_ellipsoids; j++) {
        hpp::fcl::ComputeDistance distancer(&ellipsoids[i], &ellipsoids[j]);
        for (size_t k = 0; k < num_problems_per_pair; k++) {
          M2.setRandom();
          res.clear();
          double dist = distancer(toFclTransform3f(M1), toFclTransform3f(M2), req, res);

          colbench::CollisionProblem prob;
          prob.pair_id = pair_id;
          prob.id_shape1 = i;
          prob.id_shape2 = j;
          prob.id_pose = k;
          prob.M1 = M1;
          prob.M2 = M2;
          prob.unscaled_translation = M2.translation();
          prob.unscaled_separation_vector = res.nearest_points[0] - res.nearest_points[1];
          prob.normalized_separation_vector = prob.unscaled_separation_vector.normalized();
          prob.unscaled_dist = dist;
          problems.push_back(prob);
        }
        ++pair_id;
      }
    }
    std::cout << "... DONE.\n" << std::endl;

    // Create data if does not exits
    std::string store_path_ellipsoids = std::string(COLBENCH_ELLIPSOID_DIR) + "/data";
    colbench::createDir(store_path_ellipsoids);

    // Store the convex hulls
    store_path_ellipsoids = std::string(COLBENCH_ELLIPSOID_DIR) + "/data/ellipsoids.bin";
    colbench::storeSerialize(ellipsoids, store_path_ellipsoids);

    // Store the collision problems
    std::cout << "Generated " << problems.size() << " collision problems." << std::endl;
    std::string store_path_problems = std::string(COLBENCH_ELLIPSOID_DIR) + "/data/ellipsoids_collision_problems.bin";
    colbench::storeSerialize(problems, store_path_problems);
  }
  return 0;
}
