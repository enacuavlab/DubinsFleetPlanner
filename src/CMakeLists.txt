# Copyright (c) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

#################### General configuration #################### 

if(DubinsFleetPlanner_ENABLE_ASSERTIONS)
    set(ASSERTIONS 1)
else()
    set(ASSERTIONS 0)
endif()

set(PRECISION 1e-3)
set(SOLVE_WITH_DERIVATIVES 0)
set(DEBUG_MSG 1)

if(DubinsFleetPlanner_PARALLEL)
    set(PARALLEL 1)
endif()

configure_file(ProjectHeader.h.in ProjectHeader.h)

set(DubinsFleetPlannerLib_SRC
    Aircraft.cpp
    BaseDubins.cpp
    ConflictDetection.cpp
    Dubins.cpp
    Fitting.cpp
    FleetPathPlanner.cpp
    ioUtils.cpp
    Primitives.cpp 
    utils.cpp)

set(DubinsFleetPlannerLib_INCL
    ${CMAKE_CURRENT_BINARY_DIR} 
    ${libs_dir}/eigen
    ${libs_dir}/ScalarMin
    ${libs_dir}/csv_reader)

if(DubinsFleetPlanner_PARALLEL)
    set(DubinsFleetPlannerLib_SRC
        ${DubinsFleetPlannerLib_SRC}
        ConflictMatrix.cpp)
    
    set(DubinsFleetPlannerLib_INCL
        ${DubinsFleetPlannerLib_INCL} 
        ${libs_dir}/HiGHS)
endif()

add_library(DubinsFleetPlannerLib SHARED ${DubinsFleetPlannerLib_SRC})

target_include_directories(DubinsFleetPlannerLib PUBLIC ${DubinsFleetPlannerLib_INCL})

# target_link_libraries(DubinsFleetPlanner PUBLIC Boost::headers ScalarMin)

add_executable(DubinsFleetPlanner main.cpp)
target_link_libraries(DubinsFleetPlanner PUBLIC DubinsFleetPlannerLib)
target_include_directories(DubinsFleetPlanner PUBLIC ${DubinsFleetPlannerLib_INCL})
