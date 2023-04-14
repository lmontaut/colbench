//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#include <iostream>
#include "colbench/serialization.hpp"
#include "colbench/benchmark_setup.hpp"
#include "colbench/fwd.hpp"
#include "colbench/benchmark_results.hpp"
#include "colbench/utility.hpp"
#include "benchmarks/config.h"
#include <fmt/core.h>

using colbench::CollisionProblem;
using hpp::fcl::Ellipsoid;

int main (int argc, char *argv[])
{
  po::options_description desc = colbench::createExecutableOptions(false);
  po::variables_map vm = colbench::parseExecutableArguments(argc, argv, desc);

  // Show help
  if (!colbench::showParserHelp(desc, vm)){
    // Setup benchmark according to user's input
    colbench::BenchmarkSetup bench_setup(vm);

    // Load the shapes
    std::vector<Ellipsoid> ellipsoids;
    std::string ellipsoids_path(std::string(COLBENCH_ELLIPSOID_DIR) + "/data/ellipsoids.bin");
    colbench::loadSerialize(ellipsoids, ellipsoids_path);
    std::cout << "Number of shapes: " << ellipsoids.size() << std::endl;

    // Load the collision problems
    std::vector<CollisionProblem> problems;
    std::string problems_path(std::string(COLBENCH_ELLIPSOID_DIR) + "/data/ellipsoids_collision_problems.bin");
    colbench::loadSerialize(problems, problems_path);
    std::cout << "Number of collision problems: " << problems.size() << std::endl;

    // Prepare file which stores the benchmark results
    std::ofstream file_results;
    std::string path_results(std::string(COLBENCH_ELLIPSOID_DIR) + "/data/results");
    colbench::createDir(path_results);
    auto dist_str = fmt::format("{:.0e}", bench_setup.shape_distance);
    path_results += "/" + dist_str;
    colbench::createDir(path_results);
    if (bench_setup.early_stop){
      path_results += "/boolean_collision_check";
      colbench::createDir(path_results);
    } else {
      path_results += "/distance_computation";
      colbench::createDir(path_results);
    }
    path_results += "/" + bench_setup.solver_type + ".csv";
    file_results.open(path_results);
    colbench::initResultsFile(file_results);
    colbench::BenchmarkResults bench_results;

    // create the GJK solver
    GJK gjk(bench_setup.maxit, bench_setup.tol);
    setupGJK(gjk, bench_setup);
    MinkowskiDiff mink_diff;
    Vector3d init_guess;
    support_func_guess_t init_support_guess; init_support_guess.setZero();

    // Run it for all collision problems, scaled by bench_setup.shape_distance
    Transform3f T1, T2;
    Ellipsoid* shape1; Ellipsoid* shape2;
    std::cout << "Running benchmark on all selected collision problems..." << std::endl;
    Timer timer; timer.stop();
    double timings = 0.;
    for (size_t i = 0; i < problems.size(); i++) {
      CollisionProblem& problem = problems[i];

      shape1 = &ellipsoids[problem.id_shape1];
      shape2 = &ellipsoids[problem.id_shape2];

      // Scale the problem according to user's distance input
      problem.scaleToDist(bench_setup.shape_distance);

      T1 = toFclTransform3f(problem.M1);
      T2 = toFclTransform3f(problem.M2);
      colbench::setupMinkDiffAndInitGuessAABB(mink_diff, init_guess, shape1, shape2, T1, T2, bench_setup);

      gjk.evaluate(mink_diff, init_guess, init_support_guess);

      // Store results
      colbench::appendBenchResultsToFileGJK<Ellipsoid>(bench_results, file_results, bench_setup, problem, shape1, shape2, gjk, init_guess, timings);
    }
    file_results.close();
    std::cout << "... DONE.\n" << std::endl;
  };
  return 0;
}
