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

#include <tuple>
#include <random>

#include <Eigen/Dense>
#include "Primitives.hpp"

template<DubinsMove m>
PathShape<m> generate_random_shape(int seed, double range, double max_speed)
{
    std::default_random_engine gen(seed); // Some seeded RNG 

    range = std::abs(range);
    max_speed = std::abs(max_speed);

    std::uniform_real_distribution<double> dis_pos(-range, range);
    std::uniform_real_distribution<double> dis_speedn(max_speed/10, max_speed);
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

[[gnu::const]]
inline Eigen::Vector2d angle_vector(double angle)
{
    return Eigen::Vector2d(std::cos(angle),std::sin(angle));
}

template <DubinsMove m>
inline auto samples_generator(const PathShape<m>& s);

template <>
[[gnu::pure]]
inline auto samples_generator(const PathShape<STRAIGHT>& s)
{
    Eigen::Vector2d p0(s.x,s.y);
    Eigen::Vector2d v(s.p1,s.p2);
    auto f_bind = [=](double t) {return p0 + t*v;};
    return f_bind;
}

template <DubinsMove m>
[[gnu::pure]]
inline auto samples_generator(const PathShape<m>& s)
{
    Eigen::Vector2d p0(s.x,s.y);

    double r        = s.p1;
    double omega    = s.p2;
    double phi0     = s.p4;
    auto f_bind = [=](double t) {return p0 + r*angle_vector(t*omega+phi0);};
    return f_bind;
    
}

template<DubinsMove m1, DubinsMove m2, size_t samples>
std::pair<double,double> sample_temporal_XY_dist(const PathShape<m1>& s1, const PathShape<m2>& s2, double duration)
{
    static_assert(samples > 1);
    Pose3D p1,p2;

    double t = 0.;
    p1 = follow_dubins(s1,t);
    p2 = follow_dubins(s2,t);

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;

    double min_dist = dx*dx + dy*dy;
    double min_loc = 0.;
    
    for(size_t i = 1; i < samples; i++)
    {
        t = i*duration/(samples-1);

        p1 = follow_dubins(s1,t);
        p2 = follow_dubins(s2,t);

        dx = p1.x - p2.x;
        dy = p1.y - p2.y;

        double dist = dx*dx + dy*dy;
        if (dist < min_dist)
        {
            min_dist = dist;
            min_loc = t;
        }
    }

    return {min_loc,sqrt(min_dist)};
}
