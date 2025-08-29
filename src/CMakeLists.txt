# Copyright (c) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

#################### General configuration #################### 

if(ENABLE_ASSERTIONS)
    set(ASSERTIONS 1)
else()
    set(ASSERTIONS 0)
endif()

set(PRECISION 1e-3)
set(SOLVE_WITH_DERIVATIVES 0)
set(DEBUG_MSG 1)

configure_file(ProjectHeader.h.in ProjectHeader.h)

add_library(DubinsFleetPlannerLib SHARED
    Aircraft.cpp
    BaseDubins.cpp
    ConflictDetection.cpp
    Dubins.cpp
    Fitting.cpp
    FleetPathPlanner.cpp
    ioUtils.cpp
    Primitives.cpp 
    utils.cpp)

target_include_directories(DubinsFleetPlannerLib PUBLIC 
    ${CMAKE_CURRENT_BINARY_DIR} 
    ${libs_dir}/eigen
    ${libs_dir}/ScalarMin
    ${libs_dir}/csv_reader)
# target_link_libraries(DubinsFleetPlanner PUBLIC Boost::headers ScalarMin)

add_executable(DubinsFleetPlanner main.cpp)
target_link_libraries(DubinsFleetPlanner PUBLIC DubinsFleetPlannerLib)
target_include_directories(DubinsFleetPlanner PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR} 
    ${libs_dir}/eigen
    ${libs_dir}/ScalarMin
    ${libs_dir}/csv_reader)
