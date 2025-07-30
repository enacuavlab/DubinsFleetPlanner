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

#include <limits>
#include <cmath>
#include <tuple>

#include "utils.hpp"


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
 * 
 * This file implement the length computation for these paths pased on a 'normalized' problem, where the start
 * is at (0,0) oriented with angle `alpha` (radian), the end is at (d,0) oriented with angle `beta` 
 * and the turn radius is set to 1. 
 */

double LSL_first_distance(double alpha, double beta, double d)  [[gnu::const]];
double LSL_middle_distance(double alpha, double beta, double d) [[gnu::const]];
double LSL_last_distance(double alpha, double beta, double d)   [[gnu::const]];

double LSL_total_distance(double alpha, double beta, double d)  [[gnu::const]];


double RSR_first_distance(double alpha, double beta, double d)  [[gnu::const]];
double RSR_middle_distance(double alpha, double beta, double d) [[gnu::const]];
double RSR_last_distance(double alpha, double beta, double d)   [[gnu::const]];

double RSR_total_distance(double alpha, double beta, double d)  [[gnu::const]];


double RSL_first_distance(double alpha, double beta, double d)  [[gnu::const]];
double RSL_middle_distance(double alpha, double beta, double d) [[gnu::const]];
double RSL_last_distance(double alpha, double beta, double d)   [[gnu::const]];

double RSL_total_distance(double alpha, double beta, double d)  [[gnu::const]];


double LSR_first_distance(double alpha, double beta, double d)  [[gnu::const]];
double LSR_middle_distance(double alpha, double beta, double d) [[gnu::const]];
double LSR_last_distance(double alpha, double beta, double d)   [[gnu::const]];

double LSR_total_distance(double alpha, double beta, double d)  [[gnu::const]];


double RLR_first_distance(double alpha, double beta, double d)  [[gnu::const]];
double RLR_middle_distance(double alpha, double beta, double d) [[gnu::const]];
double RLR_last_distance(double alpha, double beta, double d)   [[gnu::const]];

double RLR_total_distance(double alpha, double beta, double d)  [[gnu::const]];


double LRL_first_distance(double alpha, double beta, double d)  [[gnu::const]];
double LRL_middle_distance(double alpha, double beta, double d) [[gnu::const]];
double LRL_last_distance(double alpha, double beta, double d)   [[gnu::const]];

double LRL_total_distance(double alpha, double beta, double d)  [[gnu::const]];


double SRS_first_distance(double alpha, double beta, double d)  [[gnu::const]];
double SRS_middle_distance(double alpha, double beta, double d) [[gnu::const]];
double SRS_last_distance(double alpha, double beta, double d)   [[gnu::const]];

double SRS_total_distance(double alpha, double beta, double d)  [[gnu::const]];


double SLS_first_distance(double alpha, double beta, double d)  [[gnu::const]];
double SLS_middle_distance(double alpha, double beta, double d) [[gnu::const]];
double SLS_last_distance(double alpha, double beta, double d)   [[gnu::const]];

double SLS_total_distance(double alpha, double beta, double d)  [[gnu::const]];


/********************  General templates for Dubins functions  ********************/

/**
 * @brief The three possible moves for Dubins: Straight, Left turn and Right turn
 * 
 */
enum DubinsMove {
    STRAIGHT = 0,
    LEFT = 1,
    RIGHT = 2
};

/**
 * @brief Follow a Dubins move for a given duration
 * 
 * @tparam m            Type of move (Straight, Left turn, Right turn)
 * @param pose          Starting pose (modified on place)
 * @param duration      Move duration (s)
 * @param speed         Vehicle XY speed (m/s)
 * @param climb_rate    Vehicle Z speed ([alt]/s)
 * @param turn_radius   Vehicle XY turn radius (m)
 */
template<DubinsMove m>
void follow_dubins(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Follow a Dubins move for a given duration
 * 
 * @tparam m            Type of move (Straight, Left turn, Right turn)
 * @param pose          Starting pose 
 * @param duration      Move duration (s)
 * @param speed         Vehicle XY speed (m/s)
 * @param climb_rate    Vehicle Z speed ([alt]/s)
 * @param turn_radius   Vehicle XY turn radius (m)
 * @return Pose3d       Resulting pose
 */
template<DubinsMove m>
Pose3D follow_dubins(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius) [[gnu::pure]];


/********** Template instantiations **********/

template<>
void follow_dubins<STRAIGHT>(Pose3D* pose, double duration, double speed, double climb_rate, [[maybe_unused]] double turn_radius)
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
void follow_dubins<RIGHT>(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius)
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
void follow_dubins<LEFT>(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius)
{
    turn_left(pose,duration,speed,climb_rate,turn_radius);
}

template<>
[[gnu::pure]]
Pose3D follow_dubins<LEFT>(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    return turn_left(pose,duration,speed,climb_rate,turn_radius);
}


