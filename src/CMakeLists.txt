# Copyright (c) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

#################### General configuration #################### 

configure_file(ProjectHeader.h.in ProjectHeader.h)

add_library(DubinsFleetPlanner PUBLIC Aircraft.cpp Dubins.cpp DubinsPrimitives.cpp)
# target_link_libraries(DubinsFleetPlanner PUBLIC Boost::headers ScalarMin)
