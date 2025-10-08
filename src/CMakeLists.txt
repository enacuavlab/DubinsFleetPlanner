# Copyright (c) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

include(FetchContent)

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
    ExtendedDubins.cpp
    Fitting.cpp
    FleetPathPlanner.cpp
    FleetPlanner.cpp
    ioUtils.cpp
    Primitives.cpp 
    utils.cpp)

set(DubinsFleetPlannerLib_INCL
    ${CMAKE_CURRENT_BINARY_DIR} 
    ${libs_dir}/eigen
    ${libs_dir}/ScalarMin
    ${libs_dir}/csv_reader)

if(DubinsFleetPlanner_PARALLEL)
    ### HiGHS Solver ###

    set(HIGHS_DIR /usr/local/lib/cmake/highs)
    find_package(HIGHS REQUIRED)

    set(DubinsFleetPlannerLib_SRC
        ${DubinsFleetPlannerLib_SRC}
        ConflictList.cpp)

endif()

add_library(DubinsFleetPlannerLib SHARED ${DubinsFleetPlannerLib_SRC})

target_include_directories(DubinsFleetPlannerLib PUBLIC ${DubinsFleetPlannerLib_INCL})

if(DubinsFleetPlanner_PARALLEL)
    ### HiGHS Solver ###

    target_link_libraries(DubinsFleetPlannerLib highs::highs)

endif()

# target_link_libraries(DubinsFleetPlanner PUBLIC Boost::headers ScalarMin)

add_executable(DubinsFleetPlanner main.cpp)

target_include_directories(DubinsFleetPlanner PUBLIC ${DubinsFleetPlannerLib_INCL})
target_link_libraries(DubinsFleetPlanner PUBLIC DubinsFleetPlannerLib)