#!/bin/zsh

for dist in 1e-1 1e-2 1e-3 1e-4 -1e-4 -1e-3 -1e-2 -1e-1
do
  # Run Nesterov and Polyak
  for solver_name in nesterov polyak
  do
    for normalize_support_dir in 0 1
    do
      for early_stop in 0 1
      do
        ./build_release/benchmarks/ycb/run_ycb_benchmark --solver $solver_name --distance $dist --early_stop $early_stop --normalize_support_dir $normalize_support_dir --normalize_init_guess 0
      done
    done
  done

  # Run GJK
  for early_stop in 0 1
  do
    ./build_release/benchmarks/ycb/run_ycb_benchmark --solver default --distance $dist --early_stop $early_stop
  done
done
