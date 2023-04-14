//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#include <hpp/fcl/BV/BV.h>
#include <hpp/fcl/BV/OBB.h>
#include <iostream>
#include "colbench/benchmark_setup.hpp"
#include "colbench/serialization.hpp"
#include "colbench/fwd.hpp"
#include "colbench/benchmark_results.hpp"
#include "colbench/utility.hpp"
#include "benchmarks/config.h"
#include <fmt/core.h>
#include <stdexcept>

using colbench::CollisionProblem;
using colbench::BenchmarkSetup;
using hpp::fcl::AABB;
using hpp::fcl::OBB;

int main (int argc, char *argv[])
{
  po::options_description desc = colbench::createExecutableOptions(false);
  po::variables_map vm = colbench::parseExecutableArguments(argc, argv, desc);

  // Show help
  if (!colbench::showParserHelp(desc, vm)){
    // Setup benchmark according to user's input
    BenchmarkSetup bench_setup(vm);

    // Load the shapes
    std::vector<BVHModel<AABB>> bvhs;
    std::string shapes_path(std::string(COLBENCH_YCB_DIR) + "/data/ycb_cvx_hulls.bin");
    colbench::loadSerialize(bvhs, shapes_path);
    for (size_t i = 0; i < bvhs.size(); i++) {
      bvhs[i].buildConvexRepresentation(true);
      bvhs[i].convex->computeLocalAABB(); // Used for solver initialization
    }
    std::cout << "Number of shapes: " << bvhs.size() << std::endl;

    // Load the collision problems
    // CollisionProblem trajectories;
    // std::string trajectories_path(std::string(COLBENCH_YCB_DIR) + "/data/save.bin");
    // colbench::loadSerialize(trajectories, trajectories_path);

    std::vector<std::vector<CollisionProblem>> trajectories;
    std::string trajectories_path(std::string(COLBENCH_YCB_DIR) + "/data/ycb_trajectories.bin");
    colbench::loadSerialize(trajectories, trajectories_path);
    std::cout << "Number of trajectories: " << trajectories.size() << std::endl;

    // Prepare file which stores the benchmark results
    std::ofstream file_results;
    std::string path_results(std::string(COLBENCH_YCB_DIR) + "/data/results_trajectories");
    colbench::createDir(path_results);
    if (bench_setup.early_stop){
      path_results += "/boolean_collision_check";
      colbench::createDir(path_results);
    } else {
      path_results += "/distance_computation";
      colbench::createDir(path_results);
    }
    std::string opt = "";
    opt += "_" + colbench::WARM_START_STRATEGIES_STR[bench_setup.warm_start_strategy];
    path_results += "/" + bench_setup.solver_type + opt + ".csv";
    file_results.open(path_results);
    colbench::initResultsFile(file_results);
    colbench::BenchmarkResults bench_results;

    // create the GJK solver
    GJK gjk(bench_setup.maxit, bench_setup.tol);
    setupGJK(gjk, bench_setup);
    MinkowskiDiff mink_diff;
    Vector3d init_guess;
    support_func_guess_t init_support_guess; init_support_guess.setZero();
    support_func_guess_t init_support_guess_prev; init_support_guess_prev.setZero();

    // Run it for all collision problems, scaled by bench_setup.shape_distance
    Convex<Triangle>* shape1; Convex<Triangle>* shape2;
    std::cout << "Running benchmark on all selected collision problems..." << std::endl;
    Timer timer; timer.stop();
    size_t repetitions_per_problem = 100;
    for (size_t t = 0; t < trajectories.size(); t++) {
      // Get trajectory
      std::vector<CollisionProblem>& trajectory = trajectories[t];

      bool obb_overlap = false;
      bool obb_prev_overlap = false;

      shape1 = static_cast<Convex<Triangle>*>(bvhs[trajectory[0].id_shape1].convex.get());
      shape1->computeLocalAABB();
      AABB aabb1_loc = shape1->aabb_local;
      shape2 = static_cast<Convex<Triangle>*>(bvhs[trajectory[0].id_shape2].convex.get());
      shape2->computeLocalAABB();
      AABB aabb2_loc = shape2->aabb_local;

      init_support_guess.setZero();
      init_support_guess_prev.setZero();
      for (size_t i = 0; i < trajectory.size(); i++){
        // Get collision problem corresponding to timestep in current trajectory
        CollisionProblem& problem = trajectory[i];
        size_t iprev = std::max<size_t>(0, i-1);
        CollisionProblem& problem_prev = trajectory[iprev];

        Transform3f T1 = toFclTransform3f(problem.M1);
        Transform3f T2 = toFclTransform3f(problem.M2);

        // AABB overlap check
        OBB obb1;
        hpp::fcl::convertBV(aabb1_loc, T1, obb1);
        OBB obb2;
        hpp::fcl::convertBV(aabb2_loc, T2, obb2);
        obb_overlap = obb1.overlap(obb2);

        if (obb_overlap || !bench_setup.early_stop) {
          // Warm-starting with previous trajectory timestep
          Vector3d p1_prev = problem_prev.p1;
          Vector3d p2_prev = problem_prev.p2;
          if (bench_setup.early_stop) {
            p1_prev = problem_prev.p1_early;
            p2_prev = problem_prev.p2_early;
          }
          switch (bench_setup.warm_start_strategy){
            case BenchmarkSetup::NO_WARM_START:
              init_guess = Vector3d(1, 0, 0);
              colbench::setupMinkDiff(mink_diff, init_guess, shape1, shape2, T1, T2, bench_setup);
              break;
            case BenchmarkSetup::WARM_START_AABB:
              colbench::setupMinkDiffAndInitGuessAABB(mink_diff, init_guess, shape1, shape2, T1, T2, bench_setup);
              break;
            case BenchmarkSetup::WARM_START_PREVIOUS:
              if (iprev >= 0 && ((obb_overlap && obb_prev_overlap) || !bench_setup.early_stop)) {
                Vector3d guess = p1_prev - p2_prev;
                init_guess = T1.rotation().transpose() * guess; // Put GJK init guess in first shape referential
                colbench::setupMinkDiff(mink_diff, init_guess, shape1, shape2, T1, T2, bench_setup);
              } else {
                colbench::setupMinkDiffAndInitGuessAABB(mink_diff, init_guess, shape1, shape2, T1, T2, bench_setup);
              }
              break;
            case BenchmarkSetup::WARM_START_TRANSFORMED_PREVIOUS:
              if (iprev >= 0 && ((obb_overlap && obb_prev_overlap) || !bench_setup.early_stop)) {
                Vector3d p1_guess = problem.M1.act(problem_prev.M1.actInv(p1_prev));
                Vector3d p2_guess = problem.M2.act(problem_prev.M2.actInv(p2_prev));
                Vector3d guess = p1_guess - p2_guess;
                init_guess = T1.rotation().transpose() * guess; // Put GJK init guess in first shape referential
                colbench::setupMinkDiff(mink_diff, init_guess, shape1, shape2, T1, T2, bench_setup);
              } else {
                colbench::setupMinkDiffAndInitGuessAABB(mink_diff, init_guess, shape1, shape2, T1, T2, bench_setup);
              }
              break;
            default:
              std::logic_error("Unimplemented warm start strategy");
          }

          if (iprev >= 0) {
            init_support_guess = init_support_guess_prev;
          }

          // Evaluate solver
          double timings;
          timer.start();
          for (size_t i = 0; i < repetitions_per_problem; i++) {
            gjk.evaluate(mink_diff, init_guess, init_support_guess);
          }
          timer.stop();
          timings = timer.elapsed().user / double(repetitions_per_problem);
          init_support_guess = gjk.support_hint;

          // Store results
          colbench::appendBenchResultsToFileGJK<Convex<Triangle>>(bench_results, file_results, bench_setup, problem, shape1, shape2, gjk, init_guess, timings);
        }
        obb_prev_overlap = obb_overlap;
      }
    }
    file_results.close();
    std::cout << "... DONE.\n" << std::endl;
  }
  return 0;
}
