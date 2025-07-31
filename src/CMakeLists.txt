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

configure_file(ProjectHeader.h.in ProjectHeader.h)

add_library(DubinsFleetPlanner STATIC Aircraft.cpp BaseDubins.cpp Dubins.cpp Fitting.cpp Primitives.cpp utils.cpp)

target_include_directories(DubinsFleetPlanner PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
# target_link_libraries(DubinsFleetPlanner PUBLIC Boost::headers ScalarMin)
