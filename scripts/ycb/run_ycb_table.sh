#!/bin/zsh

for dist in 1e-2 1e-3 1e-4 -1e-4 -1e-3 -1e-2
do
  for solver_name in default polyak nesterov
  do
    for early_stop in 0 1
    do
      ./build/benchmarks/ycb/run_ycb_table --solver $solver_name --distance $dist --early_stop $early_stop
    done
  done
done
