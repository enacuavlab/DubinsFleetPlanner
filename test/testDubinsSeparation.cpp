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

#include "testDubinsSeparation.hpp"

template<size_t samples>
void sample_test_dubins_separation(const Dubins& d1, const Dubins& d2, 
    double v1, double v2, 
    double duration, double separation,
    std::string expect_msg = "")
{
    bool test_separation    = d1.is_XY_separated_from(d2,v1,v2,duration,separation);

    std::pair<double,double> sampled = sample_temporal_XY_dist<samples>(d1,d2,v1,v2,duration);
    bool check_separation   = sampled.second > separation;

    ASSERT_TRUE((test_separation == check_separation) || (!test_separation && check_separation)) << expect_msg;
    EXPECT_EQ(test_separation,check_separation) << expect_msg;
}

TEST(DubinsSeparation,RandomNoFit)
{
    for(int i = 0; i < TEST_NUM; i++)
    {
        Pose3D p1_start = generate_pose(5*i+1, TEST_POS_RANGE, 0.1);
        Pose3D p1_end   = generate_pose(5*i+2, TEST_POS_RANGE, 0.1);

        Pose3D p2_start = generate_pose(5*i+3, TEST_POS_RANGE, 0.1);
        Pose3D p2_end   = generate_pose(5*i+4, TEST_POS_RANGE, 0.1);

        std::default_random_engine gen(5*i); // Some seeded RNG 
        std::uniform_real_distribution<double> dis_speedn(TEST_SPEED_RANGE/10, TEST_SPEED_RANGE);

        double p1_speed = dis_speedn(gen);
        double p2_speed = dis_speedn(gen);
        double min_separation = TEST_MIN_SEPARATION;

        std::vector<std::unique_ptr<Dubins>> p1_possibilities = list_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,p1_start,p1_end);
        std::vector<std::unique_ptr<Dubins>> p2_possibilities = list_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,p2_start,p2_end);

        if (p1_possibilities.size() == 0)
        {
            std::cout << "Case n° " << i << "  : No path found for path 1. Skipping..." << std::endl;
            continue;
        }

        if (p2_possibilities.size() == 0)
        {
            std::cout << "Case n° " << i << "  : No path found for path 2. Skipping..." << std::endl;
            continue;
        }

        for(size_t i1 = 0; i1 < p1_possibilities.size(); i1++)
        {
            auto& d1 = p1_possibilities[i1];
            for(size_t i2 = 0; i2 < p2_possibilities.size(); i2++)
            {
                auto& d2 = p2_possibilities[i2];
                std::string expect_msg = "Failure at case n° " + std::to_string(i) + " using 1: " + d1->get_type_abbr() + " and 2: " + d2->get_type_abbr();
                double duration = std::min(d1->get_length()/p1_speed,d2->get_length()/p2_speed);
                sample_test_dubins_separation<TEST_SAMPLES_COUNT>(*d1,*d2,p1_speed,p2_speed,duration,min_separation,expect_msg);
            }
        }
    }
}

[[gnu::pure]]
static double longest_path(const std::vector<std::unique_ptr<Dubins>>& dubins_list)
{
    double max_len = 0;
    for(auto& d : dubins_list)
    {
        double len = d->get_length();
        max_len = std::max(max_len,len);
    }
    return max_len;
}

TEST(DubinsSeparation,RandomFitToLongest)
{
    for(int i = 0; i < TEST_NUM; i++)
    {
        // Generate endpoints
        Pose3D p1_start = generate_pose(5*i+1, TEST_POS_RANGE, 0.1);
        Pose3D p1_end   = generate_pose(5*i+2, TEST_POS_RANGE, 0.1);

        Pose3D p2_start = generate_pose(5*i+3, TEST_POS_RANGE, 0.1);
        Pose3D p2_end   = generate_pose(5*i+4, TEST_POS_RANGE, 0.1);

        // Generate random speeds
        std::default_random_engine gen(5*i); // Some seeded RNG 
        std::uniform_real_distribution<double> dis_speedn(TEST_SPEED_RANGE/10, TEST_SPEED_RANGE);

        double p1_speed = dis_speedn(gen);
        double p2_speed = dis_speedn(gen);
        double min_separation = TEST_MIN_SEPARATION;

        // Draw paths before fitting
        std::vector<std::unique_ptr<Dubins>> p1_possibilities = list_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,p1_start,p1_end);
        std::vector<std::unique_ptr<Dubins>> p2_possibilities = list_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,p2_start,p2_end);

        if (p1_possibilities.size() == 0)
        {
            std::cout << "Case n° " << i << "  : No path found for path 1. Skipping..." << std::endl;
            continue;
        }

        if (p2_possibilities.size() == 0)
        {
            std::cout << "Case n° " << i << "  : No path found for path 2. Skipping..." << std::endl;
            continue;
        }

        // Use longest path for fitting
        double max_duration = std::max(
            longest_path(p1_possibilities)/p1_speed,
            longest_path(p2_possibilities)/p2_speed
        );

        p1_possibilities.clear();
        p2_possibilities.clear();

        p1_possibilities = fit_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,
            p1_start, p1_end, max_duration*p1_speed, DubinsFleetPlanner_PRECISION);

        p2_possibilities = fit_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,
            p2_start, p2_end, max_duration*p2_speed, DubinsFleetPlanner_PRECISION);

        for(size_t i1 = 0; i1 < p1_possibilities.size(); i1++)
        {
            auto& d1 = p1_possibilities[i1];
            for(size_t i2 = 0; i2 < p2_possibilities.size(); i2++)
            {
                auto& d2 = p2_possibilities[i2];
                std::string expect_msg = "Failure at case n° " + std::to_string(i) + " using 1: " + d1->get_type_abbr() + " and 2: " + d2->get_type_abbr();
                double duration = std::min(d1->get_length()/p1_speed,d2->get_length()/p2_speed);
                sample_test_dubins_separation<TEST_SAMPLES_COUNT>(*d1,*d2,p1_speed,p2_speed,duration,min_separation,expect_msg);
            }
        }
    }
}