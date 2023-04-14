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

int main (int argc, char *argv[])
{
  po::options_description desc = colbench::createExecutableOptions(false);
  po::variables_map vm = colbench::parseExecutableArguments(argc, argv, desc);

  // Show help
  if (!colbench::showParserHelp(desc, vm)){
    // Setup benchmark according to user's input
    colbench::BenchmarkSetup bench_setup(vm);

    // Load the shapes
    std::vector<BVHModel<AABB>> bvhs;
    std::string shapes_path(std::string(COLBENCH_YCB_DIR) + "/data/ycb_gjk_vs_proxqp_cvx_hulls.bin");
    colbench::loadSerialize(bvhs, shapes_path);
    for (size_t i = 0; i < bvhs.size(); i++) {
      bvhs[i].buildConvexRepresentation(false);
      bvhs[i].convex->computeLocalAABB(); // Used for solver initialization
      if (bench_setup.solver_type == "proxqp"){
        Convex<Triangle>* shape;
        shape = static_cast<Convex<Triangle>*>(bvhs[i].convex.get());
        shape->buildDoubleDescription();
      }
    }
    std::cout << "Number of shapes: " << 3 << std::endl;

    // Load the collision problems
    std::vector<CollisionProblem> problems;
    std::string problems_path(std::string(COLBENCH_YCB_DIR) + "/data/ycb_gjk_vs_proxqp_collision_problems.bin");
    colbench::loadSerialize(problems, problems_path);
    std::cout << "Number of collision problems: " << problems.size() << std::endl;

    // Prepare file which stores the benchmark results
    std::ofstream file_results;
    std::string path_results(std::string(COLBENCH_YCB_DIR) + "/data/results_gjk_vs_proxqp");
    colbench::createDir(path_results);
    auto dist_str = fmt::format("{:.0e}", bench_setup.shape_distance);
    path_results += "/" + dist_str;
    colbench::createDir(path_results);
    path_results += "/distance_computation";
    colbench::createDir(path_results);
    path_results += "/" + bench_setup.solver_type + ".csv";
    file_results.open(path_results);
    colbench::initResultsFile(file_results);
    colbench::BenchmarkResults bench_results;

    // Run benchmark for all collision problems, scaled by bench_setup.shape_distance
    Transform3f T1, T2;
    Convex<Triangle>* shape1; Convex<Triangle>* shape2;
    std::cout << "Running benchmark on all selected collision problems..." << std::endl;
    Timer timer; timer.stop();
    double timings;

    if (bench_setup.solver_type == "proxqp"){
      // ----------------------
      // -- PROXQP BENCHMARK --
      // ----------------------
      size_t repetitions_per_problem = 10;
      for (size_t i = 0; i < problems.size(); i++) {
        CollisionProblem& problem = problems[i];

        // Collision pairs made of same object
        if (problem.id_shape1 == 0){
          shape1 = colbench::create_cube_proxqp(0.3, 0.3, 0.3);
        } else {
          shape1 = static_cast<Convex<Triangle>*>(bvhs[problem.id_shape1].convex.get());
        }
        if (problem.id_shape2 == 0){
          shape2 = colbench::create_cube_proxqp(0.3, 0.3, 0.3);
        } else {
          shape2 = static_cast<Convex<Triangle>*>(bvhs[problem.id_shape2].convex.get());
        }

        // Scale the problem according to user's distance input
        problem.scaleToDist(bench_setup.shape_distance);

        T1 = toFclTransform3f(problem.M1);
        T2 = toFclTransform3f(problem.M2);
        proxsuite::proxqp::dense::QP<double> qp = colbench::setupProxQPSolver(shape1, T1, shape2, T2);
        qp.settings.eps_abs = 1e-4;
        qp.settings.compute_timings = true;
        qp.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;

        timings = 0;
        for (size_t i = 0; i < repetitions_per_problem; i++) {
          qp.solve();
          timings += qp.results.info.run_time;
        }
        timings /= double(repetitions_per_problem);

        // Store results
        colbench::appendBenchResultsToFileProxQP<Convex<Triangle>>(bench_results, file_results, bench_setup, problem, shape1, shape2, qp, timings);
      }
    } else {
      // ----------------------
      // -- GJK BENCHMARK --
      // ----------------------
      // create the GJK solver
      GJK gjk(bench_setup.maxit, bench_setup.tol);
      setupGJK(gjk, bench_setup);
      MinkowskiDiff mink_diff;
      Vector3d init_guess;
      support_func_guess_t init_support_guess; init_support_guess.setZero();
      size_t repetitions_per_problem = 100;
      for (size_t i = 0; i < problems.size(); i++) {
        CollisionProblem& problem = problems[i];

        shape1 = static_cast<Convex<Triangle>*>(bvhs[problem.id_shape1].convex.get());
        shape2 = static_cast<Convex<Triangle>*>(bvhs[problem.id_shape2].convex.get());

        // Scale the problem according to user's distance input
        problem.scaleToDist(bench_setup.shape_distance);

        T1 = toFclTransform3f(problem.M1);
        T2 = toFclTransform3f(problem.M2);
        colbench::setupMinkDiffAndInitGuessAABB(mink_diff, init_guess, shape1, shape2, T1, T2, bench_setup);

        timer.start();
        for (size_t i = 0; i < repetitions_per_problem; i++) {
          gjk.evaluate(mink_diff, init_guess, init_support_guess);
        }
        timer.stop();
        timings = timer.elapsed().user / double(repetitions_per_problem);

        // Store results
        colbench::appendBenchResultsToFileGJK<Convex<Triangle>>(bench_results, file_results, bench_setup, problem, shape1, shape2, gjk, init_guess, timings);
      }
    }

    file_results.close();
    std::cout << "... DONE.\n" << std::endl;
  };
  return 0;
}
