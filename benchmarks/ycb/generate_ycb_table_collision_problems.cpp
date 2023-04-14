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

    std::cout << "Number of collision pairs: " << 6 << std::endl;
    std::cout << "Number of problems per collision pair: " << num_problems_per_pair << std::endl;
    std::cout << "Generating collision problems..." << std::endl;

    std::vector<std::shared_ptr<ConvexBase>> shapes_to_save;
    Convex<Triangle>* shape;
    for (size_t i = 0; i < 4; i++) {
      if (i == 0) {
        shape = colbench::create_cube(0.3, 0.3, 0.3);
      }
      if (i == 1) {
        shape = static_cast<Convex<Triangle>*>(shapes[26].get());
      }
      if (i == 2) {
        shape = static_cast<Convex<Triangle>*>(shapes[27].get());
      }
      if (i == 3) {
        shape = static_cast<Convex<Triangle>*>(shapes[37].get());
      }
      // Save shapes
      std::shared_ptr<ConvexBase> shape_shared_ptr;
      shape_shared_ptr.reset(shape);
      shapes_to_save.push_back(shape_shared_ptr);
    }

    size_t pair_id = 0;
    for (size_t i = 0; i < shapes_to_save.size(); i++) {
      for (size_t j = i; j < shapes_to_save.size(); j++) {
        hpp::fcl::ComputeDistance distancer(shapes_to_save[i].get(), shapes_to_save[j].get());
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
      }
      pair_id++;
    }
    std::cout << "... DONE.\n" << std::endl;

    // Store the convex hulls
    std::string store_path_cvx_hulls = std::string(COLBENCH_YCB_DIR) + "/data/ycb_table_cvx_hulls.bin";
    colbench::storeConvexHull(shapes_to_save, store_path_cvx_hulls);

    // Store the collision problems
    std::cout << "Generated " << problems.size() << " collision problems." << std::endl;
    std::string store_path_problems = std::string(COLBENCH_YCB_DIR) + "/data/ycb_table_collision_problems.bin";
    colbench::storeSerialize(problems, store_path_problems);
  }
  return 0;
}
