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


#################### Test folder ####################

########## Visual tests ##########

# if(VISUALISATON)
    include(FetchContent)

    FetchContent_Declare(
        sciplot
        GIT_REPOSITORY https://github.com/sciplot/sciplot.git
        GIT_TAG        f8d779a # release-0.3.0
        )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    FetchContent_MakeAvailable(sciplot)

    add_executable(PlotDubins plotDubins.cpp)
    target_link_libraries(PlotDubins PUBLIC sciplot DubinsFleetPlanner)
    target_include_directories(PlotDubins PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotGeometries plotGeometries.cpp)
    target_link_libraries(PlotGeometries PUBLIC sciplot DubinsFleetPlanner)
    target_include_directories(PlotGeometries PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotDubinsMoveDist plotDubinsMoveDist.cpp)
    target_link_libraries(PlotDubinsMoveDist PUBLIC sciplot DubinsFleetPlanner)
    target_include_directories(PlotDubinsMoveDist PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotDubinsWithGeometries plotDubinsWithGeometries.cpp)
    target_link_libraries(PlotDubinsWithGeometries PUBLIC sciplot DubinsFleetPlanner)
    target_include_directories(PlotDubinsWithGeometries PUBLIC "${PROJECT_SOURCE_DIR}/src")

    add_executable(PlotDubinsDist plotDubinsDist.cpp)
    target_link_libraries(PlotDubinsDist PUBLIC sciplot DubinsFleetPlanner)
    target_include_directories(PlotDubinsDist PUBLIC "${PROJECT_SOURCE_DIR}/src")
# endif()

########## Procedural tests ##########

include(GoogleTest)

macro(make_DubinsPathPlanning_test testName testFile)
    add_executable(${testName} ${testFile})
    target_link_libraries(${testName} PUBLIC DubinsFleetPlanner GTest::gtest_main)

    target_compile_options(${testName} PUBLIC
        "$<${gcc_like_cxx}:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-Wsuggest-attribute=pure;-g;-Og>"
        "$<${msvc_cxx}:-W3>"
        )

    target_include_directories(${testName} PUBLIC 
                            "${PROJECT_SOURCE_DIR}"
                            "${PROJECT_SOURCE_DIR}/src"
                            )

    gtest_discover_tests(${testName})
endmacro()

make_DubinsPathPlanning_test(TestGeometricDistance testGeometricDistance.cpp)
make_DubinsPathPlanning_test(TestTemporalDistance testTemporalDistance.cpp)
make_DubinsPathPlanning_test(TestDubinsSeparation testDubinsSeparation.cpp)




