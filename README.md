# RRT Simulator
RRT-star algorithm : https://github.com/NULL-LI/rrt-simulator/tree/rrt_star

informed-RRT-star  : https://github.com/NULL-LI/rrt-simulator/tree/informed_rrt_star

## Informed-RRT-STAR algorithm

First using rrt-connect to find a way to go. 

After that informed-rrt-star is used for optimization.

## Dependencies
* Qt5
* Eigen

## Compiling
```bash

$ cd rrt-simulator/
$ mkdir build
$ cd build
$ cmake..
$ make -j

```
Run the exectuable as
```
$ ../bin/rrt-test
```
## Interface

![RRT Simulator](imgs/rrt-sim.png)

*Note: Draw obstacles by clicking and dragging on the field.*
