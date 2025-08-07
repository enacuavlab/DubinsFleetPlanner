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
cmake_minimum_required(VERSION 3.30.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_POLICY_VERSION_MINIMUM 3.30)

set(CMAKE_VERBOSE_MAKEFILE ON)



# -------------------- Project declaration -------------------- #
project(DubinsFleetPlanner VERSION 0.1.0 LANGUAGES C CXX)

# -------------------- Compilers -------------------- #

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")

# -------------------- Required packages -------------------- #
add_subdirectory(libs)
set(libs_dir "${PROJECT_SOURCE_DIR}/libs")

### Boost ###
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

### ScalarMin solver ###
#find_package(ScalarMin REQUIRED)

# -------------------- Targets -------------------- #

add_subdirectory(src)

# -------------------- Options -------------------- #

option(ENABLE_ASSERTIONS "Enable assertion checks" ON)

option(VISUALISATION "Include visualisation in the solver (for debug/demo purpose)" ON)

# -------------------- Testing -------------------- #

include(CTest)
enable_testing()
add_subdirectory(test)
