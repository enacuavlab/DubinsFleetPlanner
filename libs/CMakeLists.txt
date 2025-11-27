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

### Eigen setup ###

if (NOT TARGET Eigen3::Eigen)
    add_subdirectory(eigen)
endif()

### nhomann JSON ###

if(DubinsFleetPlanner_USE_EXTERNAL_JSON)
    include(FetchContent)
    FetchContent_Declare(nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
        FIND_PACKAGE_ARGS NAMES nlohmann_json json)
    FetchContent_MakeAvailable(nlohmann_json)
else()
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    add_subdirectory(json)
endif()

### Others ### 

add_subdirectory(combinations)
add_subdirectory(csv_reader)

# add_subdirectory(HiGHS) # Bruteforced adding Highs by pre-compiling it
add_subdirectory(ScalarMin)