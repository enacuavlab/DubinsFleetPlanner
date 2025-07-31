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


/********************  Class implementations  ********************/

/********** LSL Lengths **********/

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::compute_fst_length(double a, double b, double d)
{
    return LSL_first_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::compute_snd_length(double a, double b, double d)
{
    return LSL_middle_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::compute_trd_length(double a, double b, double d)
{
    return LSL_last_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::compute_normalized_length(double a, double b, double d)
{
    return LSL_total_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::adjust_length(double target_length, double tol)
{
    double radius = fit_LSL(alpha,beta,d,turn_radius,target_length,tol);
    valid = !std::isnan(radius);
    turn_radius = radius;
    length = valid ? target_length : NAN;

    return radius;
}


/********** RSR Lengths **********/

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::compute_fst_length(double a, double b, double d)
{
    return RSR_first_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::compute_snd_length(double a, double b, double d)
{
    return RSR_middle_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::compute_trd_length(double a, double b, double d)
{
    return RSR_last_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::compute_normalized_length(double a, double b, double d)
{
    return RSR_total_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::adjust_length(double target_length, double tol)
{
    double radius = fit_RSR(alpha,beta,d,turn_radius,target_length,tol);
    valid = !std::isnan(radius);
    turn_radius = radius;
    length = valid ? target_length : NAN;

    return radius;
}


/********** RSL Lengths **********/

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::compute_fst_length(double a, double b, double d)
{
    return RSL_first_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::compute_snd_length(double a, double b, double d)
{
    return RSL_middle_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::compute_trd_length(double a, double b, double d)
{
    return RSL_last_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::compute_normalized_length(double a, double b, double d)
{
    return RSL_total_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::adjust_length(double target_length, double tol)
{
    double radius = fit_RSL(alpha,beta,d,turn_radius,target_length,tol);
    valid = !std::isnan(radius);
    turn_radius = radius;
    length = valid ? target_length : NAN;

    return radius;
}


/********** LSR Lengths **********/

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::compute_fst_length(double a, double b, double d)
{
    return LSR_first_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::compute_snd_length(double a, double b, double d)
{
    return LSR_middle_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::compute_trd_length(double a, double b, double d)
{
    return LSR_last_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::compute_normalized_length(double a, double b, double d)
{
    return LSR_total_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::adjust_length(double target_length, double tol)
{
    double radius = fit_LSR(alpha,beta,d,turn_radius,target_length,tol);
    valid = !std::isnan(radius);
    turn_radius = radius;
    length = valid ? target_length : NAN;

    return radius;
}


/********** RLR Lengths **********/

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::LEFT,DubinsMove::RIGHT>::compute_fst_length(double a, double b, double d)
{
    return RLR_first_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::LEFT,DubinsMove::RIGHT>::compute_snd_length(double a, double b, double d)
{
    return RLR_middle_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::LEFT,DubinsMove::RIGHT>::compute_trd_length(double a, double b, double d)
{
    return RLR_last_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::LEFT,DubinsMove::RIGHT>::compute_normalized_length(double a, double b, double d)
{
    return RLR_total_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::LEFT,DubinsMove::RIGHT>::adjust_length(double target_length, double tol)
{
    double radius = fit_RLR(alpha,beta,d,turn_radius,target_length,tol);
    valid = !std::isnan(radius);
    turn_radius = radius;
    length = valid ? target_length : NAN;

    return radius;
}


/********** LRL Lengths **********/

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::RIGHT,DubinsMove::LEFT>::compute_fst_length(double a, double b, double d)
{
    return LRL_first_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::RIGHT,DubinsMove::LEFT>::compute_snd_length(double a, double b, double d)
{
    return LRL_middle_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::RIGHT,DubinsMove::LEFT>::compute_trd_length(double a, double b, double d)
{
    return LRL_last_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::RIGHT,DubinsMove::LEFT>::compute_normalized_length(double a, double b, double d)
{
    return LRL_total_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::RIGHT,DubinsMove::LEFT>::adjust_length(double target_length, double tol)
{
    double radius = fit_LRL(alpha,beta,d,turn_radius,target_length,tol);
    valid = !std::isnan(radius);
    turn_radius = radius;
    length = valid ? target_length : NAN;

    return radius;
}


/********** SRS Lengths **********/

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::RIGHT,DubinsMove::STRAIGHT>::compute_fst_length(double a, double b, double d)
{
    return SRS_first_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::RIGHT,DubinsMove::STRAIGHT>::compute_snd_length(double a, double b, double d)
{
    return SRS_middle_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::RIGHT,DubinsMove::STRAIGHT>::compute_trd_length(double a, double b, double d)
{
    return SRS_last_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::RIGHT,DubinsMove::STRAIGHT>::compute_normalized_length(double a, double b, double d)
{
    return SRS_total_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::RIGHT,DubinsMove::STRAIGHT>::adjust_length(double target_length, double tol)
{
    double radius = fit_SRS(alpha,beta,d,turn_radius,target_length,tol);
    valid = !std::isnan(radius);
    turn_radius = radius;
    length = valid ? target_length : NAN;

    return radius;
}


/********** SLS Lengths **********/

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::LEFT,DubinsMove::STRAIGHT>::compute_fst_length(double a, double b, double d)
{
    return SLS_first_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::LEFT,DubinsMove::STRAIGHT>::compute_snd_length(double a, double b, double d)
{
    return SLS_middle_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::LEFT,DubinsMove::STRAIGHT>::compute_trd_length(double a, double b, double d)
{
    return SLS_last_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::LEFT,DubinsMove::STRAIGHT>::compute_normalized_length(double a, double b, double d)
{
    return SLS_total_distance(a,b,d);
}

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::LEFT,DubinsMove::STRAIGHT>::adjust_length(double target_length, double tol)
{
    double radius = fit_SLS(alpha,beta,d,turn_radius,target_length,tol);
    valid = !std::isnan(radius);
    turn_radius = radius;
    length = valid ? target_length : NAN;

    return radius;
}


/********** Other **********/
// We always return NAN as we don't want acceptable paths for the other cases

template<DubinsMove fst, DubinsMove snd, DubinsMove trd>
double BaseDubins<fst,snd,trd>::compute_normalized_length([[maybe_unused]] double a, [[maybe_unused]] double b , [[maybe_unused]] double d)
{
    return NAN;
}

template<DubinsMove fst, DubinsMove snd, DubinsMove trd>
double BaseDubins<fst,snd,trd>::adjust_length([[maybe_unused]] double target_length, [[maybe_unused]] double tol)
{
    valid   = false;
    length  = NAN;

    return NAN;
}

AllBaseDubins list_possible_baseDubins(const AircraftStats& stats, const Pose3D& _start, const Pose3D& _end)
{
    AllBaseDubins candidates = std::make_tuple(
        BaseDubinsLSL(stats,_start,_end),
        BaseDubinsLSR(stats,_start,_end),
        BaseDubinsRSR(stats,_start,_end),
        BaseDubinsRSL(stats,_start,_end),
        BaseDubinsRLR(stats,_start,_end),
        BaseDubinsLRL(stats,_start,_end),
        BaseDubinsSRS(stats,_start,_end),
        BaseDubinsSLS(stats,_start,_end)
    );

    return candidates;
}