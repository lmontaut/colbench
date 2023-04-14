//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#ifndef COLBENCH_BENCHMARK_SETUP_HPP
#define COLBENCH_BENCHMARK_SETUP_HPP

#include <iostream>
#include <string.h>
#include <array>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include "colbench/fwd.hpp"

namespace po = boost::program_options;

namespace colbench {

inline po::options_description createExecutableOptions(bool generate_collision_problems){
  po::options_description desc("Allowed options");
  if (!generate_collision_problems){
    desc.add_options()
      ("help", "Get help, please.")
      ("solver", boost::program_options::value<std::string>(),
       "GJK solver")
      ("normalize_support_dir", boost::program_options::value<bool>(),
       "normalize_support_dir")
      ("normalize_init_guess", boost::program_options::value<bool>(),
       "normalize_init_guess")
      ("warm_start_strategy", boost::program_options::value<std::string>(),
       "warm_start_strategy")
      ("momentum_restart", boost::program_options::value<bool>(),
       "Momentum restart yes/no")
      ("tol", boost::program_options::value<double>(),
       "Tolerance of GJK solver")
      ("maxit", boost::program_options::value<unsigned int>(),
       "Maximum number of iteration of GJK.")
      ("distance", boost::program_options::value<double>(),
       "Set shapes at given distance")
      ("early_stop", boost::program_options::value<bool>(),
       "Whether doing boolean collision check or distance computation.")
      ;
  } else {
    desc.add_options()
      ("help", "Get help, please.")
      ("nprob_per_pair", boost::program_options::value<size_t>(),
       "Number of collision problems per collision pair.")
      ("num_ellipsoids", boost::program_options::value<size_t>(),
       "Number of ellipsoids.")
      ;
  }
    return desc;
}

inline po::variables_map parseExecutableArguments(int argc, char* argv[],
                                           po::options_description desc){
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  return vm;
}

inline bool showParserHelp(const po::options_description& desc,
                    const po::variables_map& vm){
  if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;
  }
  return 0;
}

static const std::array<std::string, 4> WARM_START_STRATEGIES_STR = {
  "no_ws",
  "aabb_ws",
  "previous_iteration_ws",
  "transformed_previous_iteration_ws"
};

struct BenchmarkSetup {
  std::string solver_type;
  bool normalize_support_dir;
  bool normalize_init_guess;
  bool momentum_restart;
  double tol;
  unsigned int maxit;
  double shape_distance;
  bool early_stop;
  enum WarmStartStrategy { NO_WARM_START, WARM_START_AABB, WARM_START_PREVIOUS, WARM_START_TRANSFORMED_PREVIOUS };
  WarmStartStrategy warm_start_strategy;

  BenchmarkSetup(){}

  inline WarmStartStrategy strToWarmStartStrategy(const std::string& strat_str) {
    if (strat_str == WARM_START_STRATEGIES_STR[0]) return NO_WARM_START;
    if (strat_str == WARM_START_STRATEGIES_STR[1]) return WARM_START_AABB;
    if (strat_str == WARM_START_STRATEGIES_STR[2]) return WARM_START_PREVIOUS;
    if (strat_str == WARM_START_STRATEGIES_STR[3]) return WARM_START_TRANSFORMED_PREVIOUS;
    std::cout << "WARNING: non-implemented input for warm start strategy. Please select from: ";
    for(size_t i = 0; i < WARM_START_STRATEGIES_STR.size(); i++)
      std::cout << WARM_START_STRATEGIES_STR[i] << ",";
    std::cout << "\n";
    return NO_WARM_START;
  }
  BenchmarkSetup(const po::variables_map& vm){
    setupFromArgumentParser(vm);
  }

  inline void setupFromArgumentParser(const po::variables_map& vm){
    // -------------
    // Select solver
    // -------------
    if (vm.count("solver")) {
      solver_type = vm["solver"].as<std::string>();
    } else {
      solver_type = "default";
      std::cout << "Selecting default value for `--solver`." << std::endl;
    }
    std::cout << "Solver used: " << solver_type << std::endl;

    if (vm.count("normalize_support_dir")) {
      normalize_support_dir = vm["normalize_support_dir"].as<bool>();
    } else {
      normalize_support_dir = false;
      std::cout << "Selecting default value for `--normalize_support_dir`." << std::endl;
    }
    if (normalize_support_dir)
      solver_type += "_normalized";
    std::cout << "Normalize support direction: " << normalize_support_dir << std::endl;

    if (vm.count("normalize_init_guess")) {
      normalize_init_guess = vm["normalize_init_guess"].as<bool>();
    } else {
      normalize_init_guess = false;
      std::cout << "Selecting default value for `--normalize_init_guess`." << std::endl;
    }
    std::cout << "Normalize init guess: " << normalize_init_guess << std::endl;

    if (vm.count("warm_start_strategy")) {
      warm_start_strategy = strToWarmStartStrategy(vm["warm_start_strategy"].as<std::string>());
    } else {
      warm_start_strategy = NO_WARM_START;
      std::cout << "Selecting default value for `--warm_start_strategy`." << std::endl;
    }
    std::cout << "Warm start strategy: " << WARM_START_STRATEGIES_STR[warm_start_strategy] << std::endl;

    // -------------
    // Momentum restart
    // -------------
    if (vm.count("momentum_restart")) {
      momentum_restart = vm["momentum_restart"].as<bool>();
    } else {
      momentum_restart = false;
      std::cout << "Selecting default value for `--momentum_restart`." << std::endl;
    }
    std::cout << "Momentum restart: " << momentum_restart << std::endl;

    // -----------------------
    // Select solver tolerance
    // -----------------------
    if (vm.count("tol")) {
      tol = vm["tol"].as<double>();
    } else {
      tol = 1e-8;
      std::cout << "Selecting default value for `--gjk_tol`." << std::endl;
    }
    std::cout << "Tolerance: " << tol << std::endl;

    // ---------------------
    // Select shape distance
    // ---------------------
    if (vm.count("distance")) {
      shape_distance = vm["distance"].as<double>();
    }
    else {
      shape_distance = 1e-2;
      std::cout << "Selecting default value for `--distance`." << std::endl;
    }
    std::cout << "Shape distance: " << shape_distance << std::endl;

    // -------------------------------
    // Select max number of iterations
    // -------------------------------
    if (vm.count("maxit")) {
      maxit = vm["maxit"].as<unsigned int>();
    }
    else {
      maxit = 128;
      std::cout << "Selecting default value for `--maxit`." << std::endl;
    }
    std::cout << "Maximum number of iterations: " << maxit << std::endl;

    // -----------------------------------------------
    // Boolean collision check or distance computation
    // -----------------------------------------------
    if (vm.count("early_stop")) {
      early_stop = vm["early_stop"].as<bool>();
    }
    else {
      early_stop = false;
      std::cout << "Selecting default value for `--early_stop`." << std::endl;
    }
    if(early_stop){
      std::cout << "Running in Boolean Collision Check mode."  << std::endl;
    } else {
      std::cout << "Running in Distance Computation mode."  << std::endl;
    }
}

}; // struct BenchmarkSetup

inline void setupGJK(GJK& gjk, const colbench::BenchmarkSetup& bench_setup) {
  gjk.convergence_criterion = hpp::fcl::GJKConvergenceCriterion::DualityGap;
  gjk.convergence_criterion_type = hpp::fcl::GJKConvergenceCriterionType::Absolute;
  gjk.restart_momentum = false;
  if (bench_setup.solver_type == "default") {
    gjk.gjk_variant = hpp::fcl::GJKVariant::DefaultGJK;
  }
  else if (bench_setup.solver_type == "nesterov" || bench_setup.solver_type == "nesterov_normalized") {
    gjk.gjk_variant = hpp::fcl::GJKVariant::NesterovAcceleration;
  }
  else if (bench_setup.solver_type == "polyak" || bench_setup.solver_type == "polyak_normalized") {
    gjk.gjk_variant = hpp::fcl::GJKVariant::PolyakAcceleration;
  }
  else if (bench_setup.solver_type == "polyak_nesterov" || bench_setup.solver_type == "polyak_nesterov_normalized") {
    gjk.gjk_variant = hpp::fcl::GJKVariant::PolyakNesterovAcceleration;
  }
  else if (bench_setup.solver_type == "nesterov_polyak" || bench_setup.solver_type == "nesterov_polyak_normalized") {
    gjk.gjk_variant = hpp::fcl::GJKVariant::NesterovPolyakAcceleration;
  } else {
    throw std::logic_error(bench_setup.solver_type + " not implemented. Available options are: default, nesterov, polyak.");
  }

  if (bench_setup.early_stop){
    gjk.setDistanceEarlyBreak(0.); // Boolean collision check
  } else {
    gjk.setDistanceEarlyBreak(std::numeric_limits<double>::max());
  }
}

using hpp::fcl::ShapeBase;
using hpp::fcl::Transform3f;
inline void setupMinkDiff(MinkowskiDiff& mink_diff,
                          const ShapeBase* shape1,
                          const ShapeBase* shape2,
                          const Transform3f& T1, const Transform3f& T2,
                          const BenchmarkSetup& bench_setup)
{
  mink_diff.set(shape1, shape2, T1, T2);
  if (bench_setup.solver_type.find("normalized") != std::string::npos){
    mink_diff.normalize_support_direction = true;
  }
  else {
    mink_diff.normalize_support_direction = false;
  }
}

inline void setupMinkDiff(MinkowskiDiff& mink_diff,
                          Vector3d& init_guess,
                          const ShapeBase* shape1,
                          const ShapeBase* shape2,
                          const Transform3f& T1, const Transform3f& T2,
                          const BenchmarkSetup& bench_setup)
{
  setupMinkDiff(mink_diff, shape1, shape2, T1, T2, bench_setup);
  if (bench_setup.normalize_init_guess)
    init_guess.normalize();
}

inline void setupMinkDiffAndInitGuessAABB(MinkowskiDiff& mink_diff, Vector3d& init_guess,
                                          const ShapeBase* shape1,
                                          const ShapeBase* shape2,
                                          const Transform3f& T1, const Transform3f& T2,
                                          const BenchmarkSetup& bench_setup)
{
  setupMinkDiff(mink_diff, shape1, shape2, T1, T2, bench_setup);
  init_guess = shape1->aabb_local.center() -
    (mink_diff.oR1 * shape2->aabb_local.center() + mink_diff.ot1);
  if (bench_setup.normalize_init_guess)
    init_guess.normalize();
}

}
#endif
