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

/********************  Extra: Dubins interception with wind  ********************/

[[gnu::const]]
std::pair<double,double> min_line_interception(double x_i, double y_i, double speed, double x_T, double y_T, double vx_T, double vy_T)
{
    double dx = x_i - x_T;
    double dy = y_i - y_T;
    
    double v = speed;

    double v_norm2 = vx_T*vx_T + vy_T*vy_T;
    double w = sqrt(v_norm2);
    double v_angle = atan2(vy_T,vx_T);

    double x = cos(v_angle)*dx - sin(v_angle)*dy;
    double y = sin(v_angle)*dx + cos(v_angle)*dy;


    // They are already overlapping
    if (x < 1e-9 && y < 1e-9)
    {
        return std::make_pair(0.,0.);
    }

    double b = w*x;
    double denom = v*v - w*w;
    double discr = v*v*x*x+(v*v-w*w)*y*y;

    if (discr < 0)
    {
        return std::make_pair(NAN,NAN);
    }
        
    double tau_1 = (b - sqrt(discr))/denom;
    double tau_2 = (b + sqrt(discr))/denom;

    double tau = 0;

    if (tau_1 < 0 && tau_2 < 0)
    {
        return std::make_pair(NAN,NAN);
    }
    else if (tau_1 < 0 && tau_2 > 0)
    {
        tau = tau_2;
    }
    else if (tau_1 > 0 && tau_2 < 0)
    {
        tau = tau_1;
    }
    else
    {
        tau = std::min(tau_1,tau_2);
    }

    double A = y/(tau * v);
    double B = (x + tau * w)/(tau * v);

    double delta = atan2(A,B);

    double alpha = delta + v_angle;

    return std::make_pair(tau,alpha);
}
/*
double intercept_LSL(double x_f, double y_f, double theta_f, double wx, double wy, double rho, double speed)
{
    double vw = sqrt(wx*wx + wy*wy)/speed;

    auto A_f = [&](int k)
    {
        return x_f - rho * sin(theta_f) - wx * rho*(theta_f + 2*k*M_PI);
    };

    auto B_f = [&](int k)
    {
        return y_f - rho * (1 - cos(theta_f)) - wy * rho*(theta_f + 2*k*M_PI);
    };

    auto beta_p = [&](double A, double B)
    {
        return (sqrt((A*wx + B*wy)*(A*wx + B*wy) + (A*A+B*B)*(1-vw*vw)) - (A*wx + B*wy))/(1-vw*vw);
    };

    auto beta_m = [&](double A, double B)
    {
        return (-sqrt((A*wx + B*wy)*(A*wx + B*wy) + (A*A+B*B)*(1-vw*vw)) - (A*wx + B*wy))/(1-vw*vw);
    };

    auto alpha = [&](double A, double B, double beta)
    {
        return atan2(B-beta*wy, A-beta*wx);
    };

    auto gamma = [&](int k, double a)
    {
        return 2*k*M_PI + theta_f - a;
    };
}

double intercept_RSR(double x_f, double y_f, double theta_f, double wx, double wy, double rho, double speed)
{
    double vw = sqrt(wx*wx + wy*wy)/speed;

    auto A_f = [&](int k)
    {
        return x_f + rho * sin(theta_f) + wx * rho * (theta_f + 2*k*M_PI);
    };

    auto B_f = [&](int k)
    {
        return y_f + rho * (1 - cos(theta_f)) + wy * rho * (theta_f + 2*k*M_PI);
    };

    auto beta_p = [&](double A, double B)
    {
        return (sqrt((A*wx + B*wy)*(A*wx + B*wy) + (A*A+B*B)*(1-vw*vw)) - (A*wx + B*wy))/(1-vw*vw);
    };

    auto beta_m = [&](double A, double B)
    {
        return (-sqrt((A*wx + B*wy)*(A*wx + B*wy) + (A*A+B*B)*(1-vw*vw)) - (A*wx + B*wy))/(1-vw*vw);
    };

    auto alpha = [&](double A, double B, double b)
    {
        return atan2(-B+b*wy, A-b*wx);
    };

    auto gamma = [&](int k, double a)
    {
        return - 2*k*M_PI - theta_f - a;
    };
}

double fast_line_interception(const Pose3D& start, const Pose3D& end, double rho, double wx, double wy, double speed)
{
    double vw = sqrt(wx*wx + wy*wy)/speed;

    // Change of referential
    Pose3D target;
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double dz = end.z - start.z;

    target.theta = end.theta - start.theta;
    
    target.x = cos(-start.theta)*dx - sin(-start.theta)*dy;
    target.y = sin(-start.theta)*dx + cos(-start.theta)*dy;
    target.z = dz;

}
*/

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


template<DubinsMove m>
Pose3D follow_dubins(const PathShape<m>& s, double duration)
{
    Pose3D start;
    double speed = path_planar_speed(s);
    double climb_rate = s.p3;

    if (m == STRAIGHT)
    {
        start.x = s.x;
        start.y = s.y;
        start.z = s.z;
        start.theta = atan2(s.p2,s.p1);

        return follow_dubins<m>(start,duration,speed,climb_rate,0.);
    }
    else
    {
        start.z = s.z;

        double start_angle = s.p4;
        double turn_radius = s.p1;

        start.x = s.x + turn_radius* cos(start_angle);
        start.y = s.y + turn_radius* sin(start_angle);
        start.theta = start_angle + M_PI_2 * ((m==LEFT) ? (1) : (-1));

        return follow_dubins<m>(start,duration,speed,climb_rate,turn_radius);
    }
}

template Pose3D follow_dubins(const PathShape<STRAIGHT>&    s, double duration);
template Pose3D follow_dubins(const PathShape<RIGHT>&       s, double duration);
template Pose3D follow_dubins(const PathShape<LEFT>&        s, double duration);

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

    double d_norm = std::sqrt(dx*dx + dy*dy);
    if (d_norm < 1e-9)
    {
        output.p1 = h_speed;
        output.p2 = 0;
        output.p3 = v_speed;
    }
    else
    {
        output.p1 = h_speed*dx/d_norm;
        output.p2 = h_speed*dy/d_norm;
        output.p3 = v_speed;
    }

    output.p4 = 0.;


    return output;
}

/********************  Modify a path by shifting its starting point  ********************/

template<>
[[gnu::pure]]
PathShape<STRAIGHT> shift_start(const PathShape<STRAIGHT>& s, double duration)
{
    PathShape<STRAIGHT> output(s);

    double sx = s.x;
    double sy = s.y;
    double sz = s.z;

    output.x = sx + s.p1*duration;
    output.y = sy + s.p2*duration;
    output.z = sz + s.p3*duration;

    return output;
}

template<DubinsMove m>
[[gnu::pure]]
PathShape<m> shift_start(const PathShape<m>& s, double duration)
{
    static_assert(m != STRAIGHT);

    PathShape<m> output(s);

    output.z += s.p3*duration;

    output.p4 += s.p2*duration;

    return output;
}

template PathShape<LEFT> shift_start(const PathShape<LEFT>&, double);
template PathShape<RIGHT> shift_start(const PathShape<RIGHT>&, double);