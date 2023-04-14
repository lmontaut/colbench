#!/bin/zsh

for dist in 1e-0 1e-1 1e-2 1e-3 1e-4 -1e-4 -1e-3 -1e-2 -1e-1
do
  for solver_name in default polyak nesterov
  do
    for early_stop in 0 1
    do
      ./build/benchmarks/ellipsoids/run_ellipsoids_benchmark --solver $solver_name --distance $dist --early_stop $early_stop
    done
  done
done
