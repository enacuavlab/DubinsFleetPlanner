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

#include <set>

#include "BaseDubins.hpp"
#include "plotDubins.hpp"
#include "ConflictDetection.hpp"

#include "randomPathShape.hpp"



#ifndef PLOTTING_SAMPLES
#define PLOTTING_SAMPLES 2000
#endif

#ifndef TEST_POS_RANGE
#define TEST_POS_RANGE 5.
#endif

#ifndef TEST_SPEED_RANGE
#define TEST_SPEED_RANGE 2.
#endif

#ifndef TEST_PATH_DURATION
#define TEST_PATH_DURATION 5.5
#endif

#ifndef TEST_MIN_TURN_RADIUS
#define TEST_MIN_TURN_RADIUS 0.6
#endif


int main()
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    // -------------------- Generate test cases -------------------- //

    uint i = 10;

    Pose3D p1_start = generate_pose(5*i+1, TEST_POS_RANGE, 0.1);
    Pose3D p1_end   = generate_pose(5*i+2, TEST_POS_RANGE, 0.1);

    Pose3D p2_start = generate_pose(5*i+3, TEST_POS_RANGE, 0.1);
    Pose3D p2_end   = generate_pose(5*i+4, TEST_POS_RANGE, 0.1);

    double target_duration = 3*TEST_POS_RANGE;
    double p1_speed = 1.6;
    double p2_speed = 1.6;
    double target_length_1 = target_duration*p1_speed;
    double target_length_2 = target_duration*p2_speed;
    double min_separation = 1.;

    std::vector<std::unique_ptr<Dubins>> p1_possibilities = fit_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,
        p1_start,p1_end,target_length_1,DubinsFleetPlanner_PRECISION);
        
    std::vector<std::unique_ptr<Dubins>> p2_possibilities = fit_possible_baseDubins(1.,TEST_MIN_TURN_RADIUS,
        p2_start,p2_end,target_length_2,DubinsFleetPlanner_PRECISION);

    if (p1_possibilities.size() == 0)
    {
        std::cout << "No path found for path 1. Aborting..." << std::endl;
        exit(1);
    }

    if (p2_possibilities.size() == 0)
    {
        std::cout << "No path found for path 2. Aborting..." << std::endl;
        exit(2);
    }

    std::cout << "Random paths generation done" << std::endl;

    // -------------------- Compute groundtruth by sampling -------------------- //

    std::set<std::pair<size_t,size_t>> valid_pairs;

    for(size_t i1 = 0; i1 < p1_possibilities.size(); i1++)
    {
        auto& d1 = p1_possibilities[i1];
        for(size_t i2 = 0; i2 < p2_possibilities.size(); i2++)
        {
            auto& d2 = p2_possibilities[i2];
            std::cout << "Checking for pair " << d1->get_type_abbr() << " " << d2->get_type_abbr() << "...";
            if (d1->is_XY_separated_from(*d2,p1_speed,p2_speed,target_duration,min_separation))
            {
                std::cout << " Valid!" << std::endl
                    << "  Checking by sampling...";

                std::pair<double,double> sampled = sample_temporal_XY_dist<samples>(*d1,*d2,p1_speed,p2_speed,target_duration);

                if (sampled.second > min_separation)
                {
                    std::cout << " Valid! -> Saving the pair" << std::endl;
                    valid_pairs.insert(std::make_pair(i1,i2));
                }
                else
                {
                    std::cout << " ERROR: Found " << sampled.second << " at time " << sampled.first << std::endl
                        << "-> Intersection of type "
                        << get_DubinsMove_name(d1->get_section_type(sampled.first)) 
                        << " - " << get_DubinsMove_name(d2->get_section_type(sampled.first)) << std::endl;
                }
            }
            else
            {
                std::cout << " Conflict detected. Next" << std::endl;
            }
            std::cout << std::endl << std::endl;
        }
    }


    // -------------------- Plotting -------------------- //

    Visualisation::Plot2D plot = Visualisation::init_plot();

    for(size_t i1 = 0; i1 < p1_possibilities.size(); i1++)
    {
        auto& d1 = p1_possibilities[i1];
        Visualisation::plot_path<samples>(plot,*d1)
            .label(std::string("1: ") + d1->get_type_abbr())
            .lineColor("blue")
            .dashType(i1);
    }

    Visualisation::plot_pose(plot,p1_possibilities[0]->get_start())
        .label("1: Init")
        .lineColor("blue");

    Visualisation::plot_pose(plot,p1_possibilities[0]->get_end())
        .label("1: End")
        .lineColor("blue");

    for(size_t i2 = 0; i2 < p2_possibilities.size(); i2++)
    {
        auto& d2 = p2_possibilities[i2];
        Visualisation::plot_path<samples>(plot,*d2)
            .label(std::string("2: ") + d2->get_type_abbr())
            .lineColor("red")
            .dashType(i2);
    }

    Visualisation::plot_pose(plot,p2_possibilities[0]->get_start())
        .label("2: Init")
        .lineColor("red");

    Visualisation::plot_pose(plot,p2_possibilities[0]->get_end())
        .label("2: End")
        .lineColor("red");

    // ---------- Display ---------- //

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();

}