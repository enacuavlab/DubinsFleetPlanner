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


#include "BaseDubins.hpp"
#include "plotDubins.hpp"
#include "FleetPathPlanner.hpp"
#include "ioUtils.hpp"


#include "randomPathShape.hpp"



#ifndef PLOTTING_SAMPLES
#define PLOTTING_SAMPLES 500
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

#ifndef TEST_AIRCRAFT_NUM 
#define TEST_AIRCRAFT_NUM 21
#endif

#ifndef TEST_AIRPORT_RADIUS
#define TEST_AIRPORT_RADIUS 50 // In nautical miles (NM)
#endif

#ifndef TEST_AIRPORT_SPEED
#define TEST_AIRPORT_SPEED 4.166 // 250knots in NM/min
#endif

#ifndef TEST_AIRPORT_TURN
#define TEST_AIRPORT_TURN 1.33 // In NM, equivalent to 360° in 2min at 250knots
#endif

#ifndef TEST_AIRPORT_AIRSEP
#define TEST_AIRPORT_AIRSEP 5 // In NM
#endif

#ifndef TEST_AIRPORT_TIMESEP
#define TEST_AIRPORT_TIMESEP 1.5 // In minutes
#endif


#include "FleetDrawPrimitives.hpp"

int main()
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));


    // -------------------- Generate test cases -------------------- //
    double wind_x = 0.;//0.3;
    double wind_y = 0.;//-0.2;
    double min_sep = TEST_AIRPORT_AIRSEP;

    constexpr uint N = TEST_AIRCRAFT_NUM;
    const std::string test_name("random_to_airport_" + std::to_string(N));
    const std::string file_format("json");

    std::array<Pose3D,N> random_poses = generate_random<N>(static_cast<double>(N),1.,12);
    std::array<Pose3D,N> hchev_3 = generate_P_chevron<N,N/3>(1.5,1.2);
    std::array<Pose3D,N> hchev_3_bis = generate_P_chevron<N,N/3>(1.5,1.2);
    std::array<Pose3D,N> circ = generate_circle<N>(std::sqrt(N));
    std::array<Pose3D,N> hline = generate_hline<N>(1.5);
    std::array<Pose3D,N> hchevron = generate_hchevron<N>(1.5,1.);
    std::array<Pose3D,N> random_circle_inward = generate_random_circle_inward<N>(TEST_AIRPORT_RADIUS,0.,M_PI_4,2);
    std::array<Pose3D,N> airport_ordinal_arrivals = generate_ordinals<N>(TEST_AIRPORT_RADIUS,0.,TEST_AIRPORT_AIRSEP+1);

    std::array<Pose3D,N> airport; airport.fill(Pose3D(0,0,0,M_PI_2));

    shift_poses(hline,Pose3D(0.,0*static_cast<double>(N),0.,0.));
    shift_poses(hchevron,Pose3D(0.,1*static_cast<double>(N),0.,0.));
    shift_poses(hchev_3,Pose3D(0.,1*static_cast<double>(N),0.,0.));
    rotate_around_poses(hchev_3_bis,Pose3D(0.,0.,0.,M_PI_2));
    shift_poses(hchev_3_bis,Pose3D(static_cast<double>(N),2*static_cast<double>(N),0.,0.));

    std::array<Pose3D,N> vline = generate_hline<N>(1.5);
    rotate_around_poses(vline,Pose3D(0.,0.,0.,M_PI_2));
    shift_poses(vline,Pose3D(static_cast<double>(N),2*static_cast<double>(N),0.,0.));

    std::array<Pose3D,N> vchevron = generate_hchevron<N>(1.5,1.);
    rotate_around_poses(vchevron,Pose3D(0.,0.,0.,-M_PI_2));
    shift_poses(vchevron,Pose3D(static_cast<double>(N)*5,2*static_cast<double>(N),0.,0.));

    std::array<AircraftStats,N> stats;
    for(uint i = 0; i < N; i++)
    {
        stats[i].id = i;
        stats[i].airspeed = TEST_AIRPORT_SPEED;
        stats[i].climb = 1.;
        stats[i].turn_radius = TEST_AIRPORT_TURN;
    }


    std::array<Pose3D,N> starts = airport_ordinal_arrivals;
    std::array<Pose3D,N> ends   = airport;
    std::array<double,N-1> delta_t; delta_t.fill(TEST_AIRPORT_TIMESEP);


    // std::cout   << "Start: " << std::endl << pose_to_string(starts[0]) << std::endl
    //             << "End: " << std::endl << pose_to_string(ends[0]) << std::endl;


    // -------------------- Test solver -------------------- //

    // auto opt_result = DubinsPP::BasicDubins::synchronised_no_checks<N>(starts,ends,stats,delta_t,wind_x,wind_y);
    auto opt_result = DubinsPP::BasicDubins::synchronised_XY_checks<N>(starts,ends,stats,min_sep,delta_t,wind_x,wind_y,
        5.,1e-6,500);

    if (!opt_result.has_value())
    {
        std::cout << std::endl << "| ***** !! No solution found !! ***** |" << std::endl;
        exit(1);
    }

    auto& results = opt_result.value();    

    // -------------------- Plotting --------------------

    Visualisation::Plot2D plot = Visualisation::init_plot();

    // ---------- Individual trajectories ---------- //

    std::vector<std::unique_ptr<Dubins>> results_vec(results.size());
    for(uint i = 0; i < results.size(); i++)
    {
        results_vec[i] = std::move(results[i]);
    }

    std::vector<AircraftStats> stats_vec(stats.cbegin(),stats.cend());


    double sampled_min_dist = Visualisation::plot_multiple_paths<PLOTTING_SAMPLES>(plot,results_vec,stats_vec,wind_x,wind_y);

    // ---------- Minimal distance points ---------- //
    
    std::cout   << "Minimal sampled distance : " << sampled_min_dist << std::endl
                << "Minimal required distance: " << min_sep << std::endl;
    
    if (sampled_min_dist < min_sep)
    {
        std::cout << "***** WARNING: Minimal Sampled distance below threshold!!! *****" << std::endl << std::endl;
    }

    // ---------- Display ---------- //

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();

    // ---------- Output ---------- //

    std::ofstream output("/home/mael/Programming/DubinsFleetPlanner/" + test_name + "." + file_format);
    if (output)
    {
        if (file_format == "csv")
        {
            DubinsPP::OutputPrinter::print_paths_as_CSV(output,results_vec,stats_vec,wind_x,wind_y,PLOTTING_SAMPLES);
        }
        else if (file_format == "json")
        {
            DubinsPP::OutputPrinter::print_paths_as_JSON(output,results_vec,stats_vec,sampled_min_dist,wind_x,wind_y);
        }
        else
        {
            std::cerr << "Unknown file format: " << file_format << std::endl << "Nothing written..." << std::endl;
        }

        output.close();
    }
    else
    {
        std::cerr << "Could not create output file..." << std::endl;
    }

    

}