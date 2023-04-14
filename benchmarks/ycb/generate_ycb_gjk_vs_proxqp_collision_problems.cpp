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

using colbench::CollisionProblem;

int main (int argc, char *argv[])
{
  po::options_description desc = colbench::createExecutableOptions(true);
  po::variables_map vm = colbench::parseExecutableArguments(argc, argv, desc);

  if (!colbench::showParserHelp(desc, vm)){
    // Read the csv with the YCB path
    std::vector<std::string> paths; paths.clear();
    colbench::read_ycb_csv(paths);

    // Load the YCB shapes
    std::cout << "Loading YCB shapes" << std::endl;
    std::vector<std::shared_ptr<ConvexBase>> shapes;
    for (size_t i = 0; i < paths.size(); i++) {
      shapes.push_back(colbench::loadConvexMesh(paths[i]));
    }
    std::cout << "... DONE." << std::endl;

    // Generate collision problems
    std::vector<CollisionProblem> problems;
    const SE3 M1 = SE3::Identity(); SE3 M2;
    DistanceRequest req;
    DistanceResult res;

    size_t num_problems_per_pair;
    if (vm.count("nprob_per_pair")) {
      num_problems_per_pair = vm["nprob_per_pair"].as<std::size_t>();
    } else {
      num_problems_per_pair = 1000;
      std::cout << "Selecting default value for `--nprob_per_pair`." << std::endl;
    }

    std::cout << "Number of collision pairs: " << 3 << std::endl;
    std::cout << "Number of problems per collision pair: " << num_problems_per_pair << std::endl;
    std::cout << "Generating collision problems..." << std::endl;

    std::vector<std::shared_ptr<ConvexBase>> shapes_to_save;
    Convex<Triangle>* shape1; Convex<Triangle>* shape2;
    for (size_t i = 0; i < 3; i++) {
      if (i == 0) {
        shape1 = colbench::create_cube(0.3, 0.3, 0.3);
        shape2 = colbench::create_cube(0.3, 0.3, 0.3);
      }
      if (i == 1) {
        shape1 = static_cast<Convex<Triangle>*>(shapes[7].get());
        shape2 = static_cast<Convex<Triangle>*>(shapes[7].get());
      }
      if (i == 2) {
        shape1 = static_cast<Convex<Triangle>*>(shapes[29].get());
        shape2 = static_cast<Convex<Triangle>*>(shapes[29].get());
      }

      hpp::fcl::ComputeDistance distancer(shape1, shape2);
      for (size_t k = 0; k < num_problems_per_pair; k++) {
        M2.setRandom();
        res.clear();
        double dist = distancer(toFclTransform3f(M1), toFclTransform3f(M2), req, res);

        colbench::CollisionProblem prob;
        prob.pair_id = i;
        prob.id_shape1 = i;
        prob.id_shape2 = i;
        prob.id_pose = k;
        prob.M1 = M1;
        prob.M2 = M2;
        prob.unscaled_translation = M2.translation();
        prob.unscaled_separation_vector = res.nearest_points[0] - res.nearest_points[1];
        prob.normalized_separation_vector = prob.unscaled_separation_vector.normalized();
        prob.unscaled_dist = dist;
        problems.push_back(prob);
      }
      // Save shapes
      std::shared_ptr<ConvexBase> shape_shared_ptr;
      shape_shared_ptr.reset(shape1);
      shapes_to_save.push_back(shape_shared_ptr);
    }
    std::cout << "... DONE.\n" << std::endl;

    // Store the convex hulls
    std::string store_path_cvx_hulls = std::string(COLBENCH_YCB_DIR) + "/data/ycb_gjk_vs_proxqp_cvx_hulls.bin";
    colbench::storeConvexHull(shapes_to_save, store_path_cvx_hulls);

    // Store the collision problems
    std::cout << "Generated " << problems.size() << " collision problems." << std::endl;
    std::string store_path_problems = std::string(COLBENCH_YCB_DIR) + "/data/ycb_gjk_vs_proxqp_collision_problems.bin";
    colbench::storeSerialize(problems, store_path_problems);
  }
  return 0;
}
