# DubinsFleetPlanner

A collision-free synchronous path planner for Dubins vehicles

## Quick demo

This planner is mostly applied to fixed-wing aircraft. Two main cases can be considered:

- Fixed-wing Unmanned Aerial Vehicles (UAVs) transitioning between formations

<video controls src="doc/media/demo_reallocate.mp4" title="12 UAVs transitioning from circle to ranks formation"></video>

- Airliners approach management (demo based on the 'quickstart' dataset provided in the [`traffic`](https://traffic-viz.github.io/index.html) library; about 3h of flights landing at Paris airports in October 2021).

<video controls src="doc/media/demo_landings_x2.mp4" title="Planning Approaches at Paris airports"></video>

## Installation

### Dependencies

The build requires:

- CMake 3.28 or newer
- A C++20 compiler
- Boost 1.83 or newer (`program_options`, `chrono`, and `system`)
- Eigen3 3.3 or newer
- nlohmann-json 3.11 or newer
- GoogleTest 1.14 or newer when testing is enabled
- the `libs/ScalarMin` and `libs/HiGHS` Git submodules

On Ubuntu, the system dependencies can be installed with:

```bash
sudo apt install cmake build-essential libboost-all-dev libeigen3-dev nlohmann-json3-dev libgtest-dev
git submodule update --init libs/ScalarMin libs/HiGHS
```

### Building

```bash
cmake -S . -B build -DDubinsFleetPlanner_TESTING=ON \
  -DDubinsFleetPlanner_VISUALISATION=OFF
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Running

### Main planner

### Benchmark scripts

### Approach management using `traffic`

## Theoretical principles overview

## Cite me!