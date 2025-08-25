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

#ifndef TEST_AIRCRAFT_NUM 
#define TEST_AIRCRAFT_NUM 25 
#endif

template<uint N>
std::array<Pose3D,N> generate_hline(double sep)
{
    std::array<Pose3D,N> output;
    for(uint i = 0; i < N; i++)
    {
        output[i] = Pose3D(i*sep,0.,0.,M_PI_2);
    }

    return output;
}

std::vector<Pose3D> generate_hline(uint N, double sep)
{
    std::vector<Pose3D> output;
    for(uint i = 0; i < N; i++)
    {
        output.push_back(
            Pose3D(i*sep,0.,0.,M_PI_2)
        );
    }

    return output;
}

template<uint N>
std::array<Pose3D,N> generate_circle(double radius)
{
    std::array<Pose3D,N> output;
    double step_angle = 2*M_PI/N;
    for(uint i = 0; i < N; i++)
    {
        output[i] = Pose3D(
                radius*std::cos(i*step_angle),
                radius*std::sin(i*step_angle),
                0.,
                i*step_angle+M_PI_2);
    }

    return output;
}

std::vector<Pose3D> generate_circle(uint N, double radius)
{
    std::vector<Pose3D> output;
    double step_angle = 2*M_PI/N;
    for(uint i = 0; i < N; i++)
    {
        output.push_back(
            Pose3D(
                radius*std::cos(i*step_angle),
                radius*std::sin(i*step_angle),
                0.,
                i*step_angle+M_PI_2)
        );
    }

    return output;
}

template<class V>
void shift_poses(V& poses, Pose3D shift)
{
    for(uint i = 0; i < poses.size(); i++)
    {
        poses[i].x += shift.x;
        poses[i].y += shift.y;
        poses[i].z += shift.z;
    }
}

template<class V>
void rotate_around_poses(V& poses, Pose3D rot_ref)
{
    for(uint i = 0; i < poses.size(); i++)
    {
        poses[i].x -= rot_ref.x;
        poses[i].y -= rot_ref.y;
        poses[i].z -= rot_ref.z;
        poses[i].theta += rot_ref.theta;
        
        double new_x = -std::cos(rot_ref.theta)*poses[i].x + std::sin(rot_ref.theta)*poses[i].y;
        double new_y = -std::sin(rot_ref.theta)*poses[i].x + std::cos(rot_ref.theta)*poses[i].y;

        poses[i].x = new_x + rot_ref.x;
        poses[i].y = new_y + rot_ref.y;
    }
}

template<class V>
void swap_poses(V& poses)
{
    uint N = poses.size();
    for(uint i = 0; i < N/2; i++)
    {
        std::swap(poses[i],poses[N-1-i]);
    }
}

int main()
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));


    // -------------------- Generate test cases -------------------- //
    double wind_x = 0.2;
    double wind_y = -0.3;
    double min_sep = 0.5;

    constexpr uint N = TEST_AIRCRAFT_NUM;

    std::array<Pose3D,N> circ = generate_circle<N>(std::sqrt(N));
    std::array<Pose3D,N> hline = generate_hline<N>(1.5);

    shift_poses(hline,Pose3D(-5,10,0.,0.));

    std::array<AircraftStats,N> stats;
    for(uint i = 0; i < N; i++)
    {
        stats[i].airspeed = 1.;
        stats[i].climb = 1.;
        stats[i].turn_radius = 1.;
    }

    std::array<Pose3D,N> starts = circ;
    std::array<Pose3D,N> ends   = hline;

    std::array<double,N-1> delta_t = {0.};

    // -------------------- Test solver -------------------- //

    // auto results = DubinsPP::BasicDubins::synchronised_no_checks<N>(starts,ends,stats,delta_t,wind_x,wind_y);
    auto results = DubinsPP::BasicDubins::synchronised_XY_checks<N>(starts,ends,stats,min_sep,delta_t,wind_x,wind_y);

    // -------------------- Plotting --------------------

    Visualisation::Plot2D plot = Visualisation::init_plot();

    // ---------- Individual trajectories ---------- //

    for(uint i = 0; i < N; i++)
    {
        std::unique_ptr<Dubins>& d = results[i];
        Visualisation::plot_path<PLOTTING_SAMPLES>(plot,*d, stats[i].airspeed, wind_x, wind_y)
            .label(std::to_string(i) + std::string(": ") + d->get_type_abbr() + std::string(" ") + std::to_string(d->get_length()))
            .lineColor(Visualisation::get_color(i));

        Visualisation::plot_pose(plot,starts[i])
            .label(std::to_string(i) + std::string(": Start"))
            .lineColor(Visualisation::get_color(i));
        Visualisation::plot_pose(plot,ends[i])
            .label(std::to_string(i) + std::string(": End"))
            .lineColor(Visualisation::get_color(i));
    }

    // ---------- Minimal distance points ---------- //
    //TODO

    // ---------- Display ---------- //

    sciplot::Figure fig     = {{plot}};
    sciplot::Canvas canvas  = {{fig}};
    canvas.size(1920,1080);
    canvas.show();

}