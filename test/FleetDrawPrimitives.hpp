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

#include <algorithm>
#include <array>
#include <stdint.h>

#include "randomPathShape.hpp"
#include "Primitives.hpp"

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
                mod_2pi(i*step_angle+M_PI_2));
    }

    return output;
}

template<uint N>
std::array<Pose3D,N> generate_circle_inward(double radius)
{
    std::array<Pose3D,N> output;
    double step_angle = 2*M_PI/N;
    for(uint i = 0; i < N; i++)
    {
        output[i] = Pose3D(
                radius*std::cos(i*step_angle),
                radius*std::sin(i*step_angle),
                0.,
                mod_2pi(i*step_angle+M_PI));
    }

    return output;
}

template<uint N>
std::array<Pose3D,N> generate_hdiag(double hsep, double vsep)
{
    std::array<Pose3D,N> output;
    for(uint i = 0; i < N; i++)
    {
        output[i] = Pose3D(i*hsep,i*vsep,0.,M_PI_2);
    }

    return output;
}

template<uint N>
std::array<Pose3D,N> generate_hchevron(double hsep, double vsep)
{
    std::array<Pose3D,N> output;
    for(uint i = 0; i < N/2; i++)
    {
        output[i] = Pose3D(i*hsep,i*vsep,0.,M_PI_2);
    }
    for(uint i = N/2; i < N; i++)
    {
        output[i] = Pose3D(i*hsep,(N-i-1)*vsep,0.,M_PI_2);
    }

    return output;
}

template<uint N, uint P>
std::array<Pose3D,N> generate_P_chevron(double hsep, double vsep)
{
    std::array<Pose3D,N> output;
    uint i = 0;
    double base_sep = 0;
    while(i < N)
    {
        for(uint j = 0; j < P/2; j++)
        {
            output[i] = Pose3D(j*hsep,j*vsep+base_sep,0.,M_PI_2);
            i++;
        }
        for(uint j = P/2; j < P; j++)
        {
            output[i] = Pose3D(j*hsep,(P-j-1)*vsep+base_sep,0.,M_PI_2);
            i++;
        }

        base_sep += vsep;
    }
    return output;
}

template<uint N>
std::array<Pose3D,N> generate_random(double xy_limit, double z_limit, int seed=0)
{
    std::array<Pose3D,N> output;
    for(uint i = 0; i < N; i++)
    {
        output[i] = generate_pose(seed+i,xy_limit,z_limit);
    }
    return output;
}

template<uint N>
std::array<Pose3D,N> generate_random_circle_inward(double radius, double altitude, double orientation_eps, int seed=0)
{
    std::default_random_engine gen(seed); // Some seeded RNG 
    std::array<Pose3D,N> output;

    radius  = std::abs(radius);
    orientation_eps = std::abs(orientation_eps);

    std::uniform_real_distribution<double> dis_pos(-M_PI, M_PI);
    std::uniform_real_distribution<double> dis_angle(-orientation_eps,orientation_eps);

    for(uint i = 0; i < N; i++)
    {
        double angle = dis_pos(gen);
        double orientation = angle + M_PI + dis_angle(gen);

        output[i].x = radius*std::cos(angle);
        output[i].y = radius*std::sin(angle);
        output[i].z = altitude;
        output[i].theta = orientation;
    }

    return output;
}

template<uint N>
std::array<Pose3D,N> generate_ordinals(double radius, double altitude, double sep)
{
    std::array<Pose3D,N> output;

    for(uint i = 0; i < N; i++)
    {
        double l_radius = radius + (i/4)*sep; 
        double angle = M_PI_4 + (i%4)*M_PI_2;

        output[i].x = l_radius*std::cos(angle);
        output[i].y = l_radius*std::sin(angle);

        output[i].z = altitude;
        output[i].theta = angle - M_PI_2;
    }

    return output;
}


template<class V>
void shift_poses(V& poses, const Pose3D& shift)
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
        
        double new_x = std::cos(rot_ref.theta)*poses[i].x - std::sin(rot_ref.theta)*poses[i].y;
        double new_y = std::sin(rot_ref.theta)*poses[i].x + std::cos(rot_ref.theta)*poses[i].y;

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
