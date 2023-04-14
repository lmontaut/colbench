#!/bin/zsh

for dist in 1e-1 1e-2 1e-3 -1e-3 -1e-2 -1e-1
do
    for solver_name in default nesterov polyak proxqp
    do
        ./build/benchmarks/ycb/run_ycb_gjk_vs_proxqp --solver $solver_name --distance $dist --early_stop 0
    done
done
