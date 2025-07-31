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

#include "Primitives.hpp"


/********************  Fundamental Dubins path computing  ********************/

/**
 * There are 6 fundamental Dubins paths, and 2 extras, described using three elementary moves among
 * right turns R, left turns L and straights S
 * 
 * Fundamental:
 * - LSL, RSR (always exist)
 * - RSL, LSR (long distance paths)
 * - RLR, LRL (short distance paths)
 * 
 * Extras:
 * - SRS, SLS ("aligned" poses, their rays must intersect)
 */


/********** LSL Lengths **********/

[[gnu::const]]
double LSL_first_distance(double alpha, double beta, double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);
    double t_lsl = mod_2pi(-alpha + std::atan2(delta_cos,(d-delta_sin)));

    return t_lsl;
}

[[gnu::const]]
double LSL_middle_distance(double alpha, double beta, double d)
{
    double delta_sin = std::sin(beta)-std::sin(alpha);
    double p_lsl = std::sqrt(2+d*d-2*std::cos(alpha-beta)-2*d*delta_sin);

    return p_lsl;
}

[[gnu::const]]
double LSL_last_distance(double alpha, double beta, double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);
    double q_lsl = mod_2pi(beta - std::atan2(delta_cos,(d-delta_sin)));

    return q_lsl;
}

double LSL_small_d_approx(double alpha, double beta, [[maybe_unused]] double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);

    double t = mod_2pi(-alpha + std::atan2(delta_cos,delta_sin));
    double p = std::sqrt(2-2*std::cos(alpha-beta));
    double q = mod_2pi(beta - std::atan2(delta_cos,delta_sin));

    return t + p + q;
}


[[gnu::const]]
double LSL_total_distance(double alpha, double beta, double d)
{
    return LSL_first_distance(alpha,beta,d) 
        + LSL_middle_distance(alpha,beta,d) 
        + LSL_last_distance(alpha,beta,d); 
}

/********** RSR Lengths **********/

[[gnu::const]]
double RSR_first_distance(double alpha, double beta, double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);
    return mod_2pi(alpha - std::atan2(-delta_cos,(d+delta_sin)));
}

[[gnu::const]]
double RSR_middle_distance(double alpha, double beta, double d)
{
    double delta_sin = std::sin(beta)-std::sin(alpha);
    return std::sqrt(2+d*d-2*std::cos(alpha-beta)+2*d*delta_sin);
}

[[gnu::const]]
double RSR_last_distance(double alpha, double beta, double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);
    return mod_2pi(- beta + std::atan2(-delta_cos,(d+delta_sin)));
}

double RSR_small_d_approx(double alpha, double beta, [[maybe_unused]] double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);

    double t = mod_2pi(alpha - std::atan2(-delta_cos,delta_sin));
    double p = std::sqrt(2-2*std::cos(alpha-beta));
    double q = mod_2pi(- beta + std::atan2(-delta_cos,delta_sin));

    return t + p + q;
}


[[gnu::const]]
double RSR_total_distance(double alpha, double beta, double d)
{
    return RSR_first_distance(alpha,beta,d) 
        + RSR_middle_distance(alpha,beta,d) 
        + RSR_last_distance(alpha,beta,d); 
}

/********** RSL Lengths **********/

[[gnu::const]]
double RSL_first_distance(double alpha, double beta, double d)
{
    double sum_cos = std::cos(alpha)+std::cos(beta);
    double sum_sin = std::sin(alpha)+std::sin(beta);
    return mod_2pi(alpha-std::atan2(sum_cos,(d-sum_sin)) + std::atan2(2,RSL_middle_distance(alpha,beta,d)));
}

[[gnu::const]]
double RSL_middle_distance(double alpha, double beta, double d)
{
    double sum_sin = std::sin(alpha)+std::sin(beta);
    return std::sqrt(d*d-2+2*std::cos(alpha-beta)-2*d*sum_sin);
}

[[gnu::const]]
double RSL_last_distance(double alpha, double beta, double d)
{
    double sum_cos = std::cos(alpha)+std::cos(beta);
    double sum_sin = std::sin(alpha)+std::sin(beta);
    return mod_2pi(beta - std::atan2(sum_cos,(d-sum_sin)) + std::atan2(2,RSL_middle_distance(alpha,beta,d)));
}

double RSL_small_d_approx(double alpha, double beta, [[maybe_unused]] double d)
{
    double sum_cos = std::cos(alpha)+std::cos(beta);
    double sum_sin = std::sin(alpha)+std::sin(beta);

    double p = std::sqrt(-2+2*std::cos(alpha-beta));

    double t = mod_2pi(alpha - std::atan2(sum_cos,sum_sin) + std::atan2(2,p));
    double q = mod_2pi(beta  - std::atan2(sum_cos,sum_sin) + std::atan2(2,p));

    return t + p + q;
}


[[gnu::const]]
double RSL_total_distance(double alpha, double beta, double d)
{
    return RSL_first_distance(alpha,beta,d) 
        + RSL_middle_distance(alpha,beta,d) 
        + RSL_last_distance(alpha,beta,d); 
}

/********** LSR Lengths **********/

[[gnu::const]]
double LSR_first_distance(double alpha, double beta, double d)
{
    double sum_cos = std::cos(alpha)+std::cos(beta);
    double sum_sin = std::sin(alpha)+std::sin(beta);
    return mod_2pi(-alpha+std::atan2(-sum_cos,(d+sum_sin))-std::atan2(-2,LSR_middle_distance(alpha,beta,d)));
}

[[gnu::const]]
double LSR_middle_distance(double alpha, double beta, double d)
{
    double sum_sin = std::sin(alpha)+std::sin(beta);
    return std::sqrt(-2+d*d+2*std::cos(alpha-beta)+2*d*sum_sin);
}

[[gnu::const]]
double LSR_last_distance(double alpha, double beta, double d)
{
    double sum_cos = std::cos(alpha)+std::cos(beta);
    double sum_sin = std::sin(alpha)+std::sin(beta);
    return mod_2pi(-beta + std::atan2(-sum_cos,(d+sum_sin))-std::atan2(-2,LSR_middle_distance(alpha,beta,d)));
}


double LSR_small_d_approx(double alpha, double beta, [[maybe_unused]] double d)
{
    double sum_cos = std::cos(alpha)+std::cos(beta);
    double sum_sin = std::sin(alpha)+std::sin(beta);

    double p = std::sqrt(-2+2*std::cos(alpha-beta));

    double t = mod_2pi(-alpha + std::atan2(-sum_cos,sum_sin)-std::atan2(-2,p));
    double q = mod_2pi(-beta  + std::atan2(-sum_cos,sum_sin)-std::atan2(-2,p));

    return t + p + q;
}


[[gnu::const]]
double LSR_total_distance(double alpha, double beta, double d)
{
    return LSR_first_distance(alpha,beta,d) 
        + LSR_middle_distance(alpha,beta,d) 
        + LSR_last_distance(alpha,beta,d); 
}

/********** RLR Lengths **********/

[[gnu::const]]
double RLR_first_distance(double alpha, double beta, double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);
    return mod_2pi(alpha - std::atan2(-delta_cos,(d+delta_sin)) + RLR_middle_distance(alpha,beta,d)/2);
}

[[gnu::const]]
double RLR_middle_distance(double alpha, double beta, double d)
{
    double delta_sin = std::sin(beta)-std::sin(alpha);
    return std::acos((6-d*d+2*std::cos(alpha-beta)-2*d*delta_sin)/8);
}

[[gnu::const]]
double RLR_last_distance(double alpha, double beta, double d)
{
    return mod_2pi(alpha-beta-RLR_first_distance(alpha,beta,d)+RLR_middle_distance(alpha,beta,d));
}

double RLR_small_d_approx(double alpha, double beta, [[maybe_unused]] double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);

    double p = std::acos((6+2*std::cos(alpha-beta))/8);

    double t = mod_2pi(alpha - std::atan2(-delta_cos,delta_sin) + p/2);

    double q = mod_2pi(alpha-beta-t+p);

    return t + p + q;
}


[[gnu::const]]
double RLR_total_distance(double alpha, double beta, double d)
{
    return RLR_first_distance(alpha,beta,d) 
        + RLR_middle_distance(alpha,beta,d) 
        + RLR_last_distance(alpha,beta,d); 
}

/********** LRL Lengths **********/

[[gnu::const]]
double LRL_first_distance(double alpha, double beta, double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);
    return mod_2pi(-alpha+std::atan2(delta_cos,(d-delta_sin)) + LRL_middle_distance(alpha,beta,d)/2);
}

[[gnu::const]]
double LRL_middle_distance(double alpha, double beta, double d)
{
    double delta_sin = std::sin(beta)-std::sin(alpha);
    return std::acos((6-d*d+2*std::cos(alpha-beta)+2*d*delta_sin)/8);
}

[[gnu::const]]
double LRL_last_distance(double alpha, double beta, double d)
{
    return mod_2pi(beta-alpha+LRL_middle_distance(alpha,beta,d) - LRL_first_distance(alpha,beta,d));
}

double LRL_small_d_approx(double alpha, double beta, [[maybe_unused]] double d)
{
    double delta_cos = std::cos(beta)-std::cos(alpha);
    double delta_sin = std::sin(beta)-std::sin(alpha);

    double p = std::acos((6+2*std::cos(alpha-beta))/8);

    double t = mod_2pi(-alpha+std::atan2(delta_cos,delta_sin) + p/2);
    
    double q = mod_2pi(beta-alpha+t-p);

    return t + p + q;
}


[[gnu::const]]
double LRL_total_distance(double alpha, double beta, double d)
{
    return LRL_first_distance(alpha,beta,d) 
        + LRL_middle_distance(alpha,beta,d) 
        + LRL_last_distance(alpha,beta,d); 
}

/********** SRS Lengths **********/

[[gnu::const]]
double SRS_first_distance(double alpha, double beta, double d)
{
    // We don't care about the special case where alpha=beta=0 because then the solution
    // is a straight line, which will overlap with LSL
    double da = central_angle(alpha-beta);
    if (std::abs(da) - M_PI == 0)
    {
        return NAN;
    }

    double cs = d * std::abs(std::sin(beta)/std::sin(da));

    double output = cs - 1/std::tan((M_PI-da)/2);  
    return (output >= 0) ? output : NAN;
}

[[gnu::const]]
double SRS_middle_distance(double alpha, double beta, double d)
{
    return mod_2pi(alpha-beta);
}

[[gnu::const]]
double SRS_last_distance(double alpha, double beta, double d)
{
    double da = central_angle(alpha-beta);
    if (std::abs(da) - M_PI == 0)
    {
        return NAN;
    }
    
    double ec = d * std::abs(std::sin(alpha)/std::sin(da));

    double output = ec - 1/std::tan((M_PI-da)/2);
    return (output >= 0) ? output : NAN;
}


[[gnu::const]]
double SRS_total_distance(double alpha, double beta, double d)
{
    return SRS_first_distance(alpha,beta,d) 
        + SRS_middle_distance(alpha,beta,d) 
        + SRS_last_distance(alpha,beta,d); 
}




/********** SLS Lengths **********/

[[gnu::const]]
double SLS_first_distance(double alpha, double beta, double d)
{
    double da = central_angle(beta-alpha);
    if (std::abs(da) - M_PI == 0)
    {
        return NAN;
    }
    
    double cs = d * std::abs(std::sin(beta)/std::sin(da));

    double output = cs - 1/std::tan((M_PI-da)/2);
    return (output >= 0) ? output : NAN;
}

[[gnu::const]]
double SLS_middle_distance(double alpha, double beta, double d)
{
    return mod_2pi(beta-alpha);
}

[[gnu::const]]
double SLS_last_distance(double alpha, double beta, double d)
{
    double da = central_angle(beta-alpha);
    if (std::abs(da) - M_PI == 0)
    {
        return NAN;
    }
    
    double ec = d * std::abs(std::sin(alpha)/std::sin(da));

    double output = ec - 1/std::tan((M_PI-da)/2);
    return (output >= 0) ? output : NAN;
}


[[gnu::const]]
double SLS_total_distance(double alpha, double beta, double d)
{
    return SLS_first_distance(alpha,beta,d) 
        + SLS_middle_distance(alpha,beta,d) 
        + SLS_last_distance(alpha,beta,d); 
}


/******************** Template specialization for Dubins moves ********************/

template<>
void follow_dubins<STRAIGHT>(Pose3D* pose, double duration, double speed, double climb_rate, [[maybe_unused]] double turn_radius)
{
    move_straight(pose,duration,speed,climb_rate);
}

template<>
[[gnu::pure]]
Pose3D update_dubins<STRAIGHT>(const Pose3D& pose, double duration, double speed, double climb_rate, [[maybe_unused]] double turn_radius)
{
    return move_straight(pose,duration,speed,climb_rate);
}

template<>
void follow_dubins<RIGHT>(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius)
{
    turn_right(pose,duration,speed,climb_rate,turn_radius);
}

template<>
[[gnu::pure]]
Pose3D update_dubins<RIGHT>(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    return turn_right(pose,duration,speed,climb_rate,turn_radius);
}

template<>
void follow_dubins<LEFT>(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius)
{
    turn_left(pose,duration,speed,climb_rate,turn_radius);
}

template<>
[[gnu::pure]]
Pose3D update_dubins<LEFT>(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    return turn_left(pose,duration,speed,climb_rate,turn_radius);
}


