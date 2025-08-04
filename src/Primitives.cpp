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

[[gnu::const]]
std::pair<double,double> LSL_possible_d(double alpha, double beta)
{
    // Limitations may come from the sqrt in the `LSL_middle_distance`,
    // but one can show it is always well-defined. Still, distance must be positive.

    return std::pair(0.,INFINITY);
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

[[gnu::const]]
std::pair<double,double> RSR_possible_d(double alpha, double beta)
{
    // Limitations may come from the sqrt in the `RSR_middle_distance`,
    // but one can show it is always well-defined. Still, distance must be positive.

    return std::pair(0.,INFINITY);
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

[[gnu::const]]
std::pair<double,double> RSL_possible_d(double alpha, double beta)
{
    // Limitations may come from the sqrt in the `RSL_middle_distance`
    double sum_sin = std::sin(alpha)+std::sin(beta);
    // One can show that the discriminant is always positive, and roots have different signs
    // Hence we only focus on the largest to get a lower bound on d
    double delta_r = sum_sin*sum_sin - 2*(std::cos(alpha-beta)-1);
    
    double d_min = sum_sin + std::sqrt(delta_r);
    d_min = std::max(0.,d_min);

    return std::pair(d_min,INFINITY);
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

[[gnu::const]]
std::pair<double,double> LSR_possible_d(double alpha, double beta)
{
    // Limitations may come from the sqrt in the `LSR_middle_distance`
    double sum_sin = std::sin(alpha)+std::sin(beta);
    // One can show that the discriminant is always positive, and roots have different signs
    // Hence we only focus on the largest to get a lower bound on d
    double delta_r = sum_sin*sum_sin - 2*(std::cos(alpha-beta)-1);
    
    double d_min = - sum_sin + std::sqrt(delta_r);
    d_min = std::max(0.,d_min);

    return std::pair(d_min,INFINITY);
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

[[gnu::const]]
std::pair<double,double> RLR_possible_d(double alpha, double beta)
{
    // Limitations may come from the acos in the `RLR_middle_distance`
    double delta_sin = std::sin(beta)-std::sin(alpha);
    // One can show that the polynomial is always less than 1

    // One can show that the discriminant is always positive, and roots have different signs
    // Hence we only focus on the largest to get a lower bound on d
    double delta_r = delta_sin*delta_sin + (2*std::cos(alpha-beta)+14);
    
    double d_min_p = std::max(0.,- delta_sin + std::sqrt(delta_r));
    double d_min_m = std::max(0.,- delta_sin - std::sqrt(delta_r));

    return std::pair(d_min_m, d_min_p);
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

[[gnu::const]]
std::pair<double,double> LRL_possible_d(double alpha, double beta)
{
    // Limitations may come from the acos in the `RLR_middle_distance`
    double delta_sin = std::sin(beta)-std::sin(alpha);
    // One can show that the polynomial is always less than 1

    // One can show that the discriminant is always positive, and roots have different signs
    // Hence we only focus on the largest to get a lower bound on d
    double delta_r = delta_sin*delta_sin + (2*std::cos(alpha-beta)+14);
    
    double d_min_p = std::max(0.,delta_sin + std::sqrt(delta_r));
    double d_min_m = std::max(0.,delta_sin - std::sqrt(delta_r));

    return std::pair(d_min_m, d_min_p);
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
    // is a straight line, which will overlap with RSR
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
std::pair<double,double> SRS_possible_d(double alpha, double beta)
{
    double da = central_angle(alpha-beta);
    // Case with nothing possible
    if (std::abs(da) - M_PI == 0)
    {
        return std::pair(INFINITY,-INFINITY);
    }
    double sC = std::abs(std::sin(beta)/std::sin(da));
    double eC = std::abs(std::sin(alpha)/std::sin(da));

    double A = 1/std::tan((M_PI-da)/2);

    double d_min = std::max(A/sC, A/eC);
    d_min = std::max(d_min, 0.);

    return std::pair(d_min,INFINITY);
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
std::pair<double,double> SLS_possible_d(double alpha, double beta)
{
    double da = central_angle(alpha-beta);
    // Case with nothing possible
    if (std::abs(da) - M_PI == 0)
    {
        return std::pair(INFINITY,-INFINITY);
    }
    double sC = std::abs(std::sin(beta)/std::sin(da));
    double eC = std::abs(std::sin(alpha)/std::sin(da));

    double A = 1/std::tan((M_PI-da)/2);

    double d_min = std::max(A/sC, A/eC);
    d_min = std::max(d_min, 0.);

    return std::pair(d_min,INFINITY);
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
void update_dubins<STRAIGHT>(Pose3D* pose, double duration, double speed, double climb_rate, [[maybe_unused]] double turn_radius)
{
    move_straight(pose,duration,speed,climb_rate);
}

template<>
[[gnu::pure]]
Pose3D follow_dubins<STRAIGHT>(const Pose3D& pose, double duration, double speed, double climb_rate, [[maybe_unused]] double turn_radius)
{
    return move_straight(pose,duration,speed,climb_rate);
}

template<>
void update_dubins<RIGHT>(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius)
{
    turn_right(pose,duration,speed,climb_rate,turn_radius);
}

template<>
[[gnu::pure]]
Pose3D follow_dubins<RIGHT>(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    return turn_right(pose,duration,speed,climb_rate,turn_radius);
}

template<>
void update_dubins<LEFT>(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius)
{
    turn_left(pose,duration,speed,climb_rate,turn_radius);
}

template<>
[[gnu::pure]]
Pose3D follow_dubins<LEFT>(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    return turn_left(pose,duration,speed,climb_rate,turn_radius);
}


/********************  Recover path shape from two points and a hint  ********************/

template<>
PathShape<LEFT> compute_params<LEFT>(const Pose3D& start, const Pose3D& end, double h_speed, double turn_radius, double v_speed)
{
    PathShape<LEFT> output;
    double ox,oy;

    ox = turn_radius*std::cos(start.theta+M_PI_2);
    oy = turn_radius*std::sin(start.theta+M_PI_2);

    output.x = start.x+ox;
    output.y = start.y+oy;
    output.z = start.z;

    output.p1 = turn_radius;
    output.p2 = h_speed/turn_radius;
    output.p3 = v_speed;
    output.p4 = start.theta-M_PI_2;

    return output;
}

template<>
PathShape<RIGHT> compute_params<RIGHT>(const Pose3D& start, const Pose3D& end, double h_speed, double turn_radius, double v_speed)
{
    PathShape<RIGHT> output;
    double ox,oy;

    ox = turn_radius*std::cos(start.theta-M_PI_2);
    oy = turn_radius*std::sin(start.theta-M_PI_2);

    output.x = start.x+ox;
    output.y = start.y+oy;
    output.z = start.z;

    output.p1 = turn_radius;
    output.p2 = -h_speed/turn_radius;
    output.p3 = v_speed;
    output.p4 = start.theta+M_PI_2;

    return output;
}

template<>
PathShape<STRAIGHT> compute_params<STRAIGHT>(const Pose3D& start, const Pose3D& end, double h_speed, [[maybe_unused]] double turn_radius, double v_speed)
{
    PathShape<STRAIGHT> output;
    output.x = start.x;
    output.y = start.y;
    output.z = start.z;

    double dx,dy,dz;
    dx = end.x - start.x;
    dy = end.y - start.y;
    dz = end.z - start.z;

    double d_norm = std::sqrt(dx*dx + dy*dy + dz*dz);
    output.p1 = h_speed*dx/d_norm;
    output.p2 = h_speed*dy/d_norm;
    output.p3 = v_speed*dz/d_norm;

    return output;
}