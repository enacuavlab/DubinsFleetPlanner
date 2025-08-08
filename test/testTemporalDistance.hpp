// Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
// 
// This file is part of DubinsFleetPlanner.
// 
// DubinsFleetPlanner is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// DubinsFleetPlanner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <random>
#include <tuple>

#include "ConflictDetection.hpp"

#ifndef TEST_NUM
#define TEST_NUM 5
#endif

#ifndef TEST_POS_RANGE
#define TEST_POS_RANGE 5
#endif

#ifndef TEST_SPEED_RANGE
#define TEST_SPEED_RANGE 1
#endif

#ifndef TEST_PATH_DURATION
#define TEST_PATH_DURATION 2
#endif

template<DubinsMove m>
PathShape<m> generate_random_shape(int seed, double range, double max_speed)
{
    std::default_random_engine gen(seed); // Some seeded RNG 

    range = std::abs(range);
    max_speed = std::abs(max_speed);

    std::uniform_real_distribution<double> dis_pos(-range, range);
    std::uniform_real_distribution<double> dis_speedn(max_speed/1e-3, max_speed);
    std::uniform_real_distribution<double> dis_angle(-M_PI,M_PI);

    PathShape<m> output;
    output.x = dis_pos(gen);
    output.y = dis_pos(gen);
    output.z = dis_pos(gen);

    output.p3 = dis_speedn(gen)/10 * (dis_pos(gen) > 0 ? 1 : -1);

    double speed_n = dis_speedn(gen);

    double angle = dis_angle(gen);

    if (m == STRAIGHT)
    {
        output.p1 = speed_n * std::cos(angle);
        output.p2 = speed_n * std::sin(angle);
    }
    else
    {
        output.p1 = dis_speedn(gen);
        output.p2 = speed_n/output.p1 * ((m == LEFT) ? 1 : -1);
        output.p4 = angle;
    }

    return output;
}