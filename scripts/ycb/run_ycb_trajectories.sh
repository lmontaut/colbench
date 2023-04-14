#!/bin/zsh

# for early_stop in 0 1
for early_stop in 0
do
  for warm_start_strategy in no_ws aabb_ws previous_iteration_ws transformed_previous_iteration_ws
  do
    # POLYAK
    ./build_release/benchmarks/ycb/run_ycb_trajectories_benchmark --solver polyak --normalize_support_dir 0 --early_stop $early_stop --warm_start_strategy $warm_start_strategy

    # NESTEROV
    ./build_release/benchmarks/ycb/run_ycb_trajectories_benchmark --solver nesterov --normalize_support_dir 0 --early_stop $early_stop --warm_start_strategy $warm_start_strategy
    ./build_release/benchmarks/ycb/run_ycb_trajectories_benchmark --solver nesterov --normalize_support_dir 1 --early_stop $early_stop --warm_start_strategy $warm_start_strategy

    # GJK
    ./build_release/benchmarks/ycb/run_ycb_trajectories_benchmark --solver default --early_stop $early_stop --warm_start_strategy $warm_start_strategy
  done
done
