# Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of DubinsFleetPlanner.
# 
# DubinsFleetPlanner is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DubinsFleetPlanner is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

# CMake general configuration
cmake_minimum_required(VERSION 3.28.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_POLICY_VERSION_MINIMUM 3.28)

# -------------------- Project declaration -------------------- #
project(DubinsFleetPlanner VERSION 0.1.0 LANGUAGES C CXX)

# -------------------- Compilers -------------------- #

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")

# -------------------- Required packages -------------------- #
### Boost ###

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.83 CONFIG REQUIRED)
find_package(Eigen3 3.3 CONFIG REQUIRED NO_MODULE)
find_package(nlohmann_json 3.11 CONFIG REQUIRED)

### Add lib subdirectory ###

add_subdirectory(libs)

set(libs_dir "${PROJECT_SOURCE_DIR}/libs")

# -------------------- Options -------------------- #

option(DubinsFleetPlanner_PARALLEL "Enable parallel solver (Requires HiGHS submodule)" ON)

option(DubinsFleetPlanner_ENABLE_ASSERTIONS "Enable assertion checks" ON)

option(DubinsFleetPlanner_TESTING "Enable testing for path planning (requires system GTest)" ON)
option(DubinsFleetPlanner_VISUALISATION "Build visualisation test tools (will download Sciplot)" OFF)

# -------------------- Targets -------------------- #

add_subdirectory(src)


option(DubinsFleetPlanner_COVERAGE "Perform coverage analysis while running tests" OFF)

# -------------------- Testing -------------------- #

if(DubinsFleetPlanner_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()
