# Copyright (c) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

find_package(Boost CONFIG REQUIRED COMPONENTS program_options chrono system)


#################### General configuration ####################

if(DubinsFleetPlanner_ENABLE_ASSERTIONS)
    set(ASSERTIONS 1)
else()
    set(ASSERTIONS 0)
endif()

set(PRECISION 1e-2)
set(ZERO_TOLERANCE 1e-9)
set(SOLVE_WITH_DERIVATIVES 0)
set(DEBUG_MSG 1)

if(DubinsFleetPlanner_PARALLEL)
    set(PARALLEL 1)
endif()

configure_file(ProjectHeader.h.in ProjectHeader.h)

set(DubinsFleetPlannerLib_SRC
    FleetPlanner.cpp
    Aircraft.cpp
    BaseDubins.cpp
    ConflictDetection.cpp
    ConflictList.cpp
    Dubins.cpp
    Ellipse.cpp
    ExtendedDubins.cpp
    Fitting.cpp
    ioUtils.cpp
    PathGenerators/Extended.cpp
    PathGenerators/Sampling.cpp
    Primitives.cpp
    Pose3D.cpp
    utils.cpp)

set(DubinsFleetPlannerLib_INCL
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${libs_dir}/ScalarMin
    ${libs_dir}/csv_reader)

### HiGHS Solver ###

if(NOT EXISTS "${libs_dir}/HiGHS/CMakeLists.txt")
    message(FATAL_ERROR
        "HiGHS source is missing. Initialize the submodule with "
        "'git submodule update --init libs/HiGHS'.")
endif()

set(BUILD_CXX_EXE OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
add_subdirectory(${libs_dir}/HiGHS ${CMAKE_BINARY_DIR}/HiGHS EXCLUDE_FROM_ALL)

add_library(DubinsFleetPlannerLib SHARED ${DubinsFleetPlannerLib_SRC})

target_include_directories(DubinsFleetPlannerLib PUBLIC ${DubinsFleetPlannerLib_INCL})

target_link_libraries(DubinsFleetPlannerLib
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    highs::highs
    Boost::program_options
    Boost::chrono
    Boost::system)

# target_link_libraries(DubinsFleetPlanner PUBLIC Boost::headers ScalarMin)

add_executable(DubinsFleetPlanner main.cpp)

target_include_directories(DubinsFleetPlanner PUBLIC ${DubinsFleetPlannerLib_INCL})
target_link_libraries(DubinsFleetPlanner LINK_PUBLIC DubinsFleetPlannerLib)
