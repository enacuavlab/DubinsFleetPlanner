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
#include <array>
#include <string>

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

double LSL_first_distance(double alpha, double beta, double d);
double LSL_middle_distance(double alpha, double beta, double d);
double LSL_last_distance(double alpha, double beta, double d);

double LSL_total_distance(double alpha, double beta, double d);
std::pair<double,double> LSL_possible_d(double alpha, double beta);


double RSR_first_distance(double alpha, double beta, double d);
double RSR_middle_distance(double alpha, double beta, double d);
double RSR_last_distance(double alpha, double beta, double d);

double RSR_total_distance(double alpha, double beta, double d);
std::pair<double,double> RSR_possible_d(double alpha, double beta);


double RSL_first_distance(double alpha, double beta, double d);
double RSL_middle_distance(double alpha, double beta, double d);
double RSL_last_distance(double alpha, double beta, double d);

double RSL_total_distance(double alpha, double beta, double d);
std::pair<double,double> RSL_possible_d(double alpha, double beta);


double LSR_first_distance(double alpha, double beta, double d);
double LSR_middle_distance(double alpha, double beta, double d);
double LSR_last_distance(double alpha, double beta, double d);

double LSR_total_distance(double alpha, double beta, double d);
std::pair<double,double> LSR_possible_d(double alpha, double beta);


double RLR_first_distance(double alpha, double beta, double d);
double RLR_middle_distance(double alpha, double beta, double d);
double RLR_last_distance(double alpha, double beta, double d);

double RLR_total_distance(double alpha, double beta, double d);
std::pair<double,double> RLR_possible_d(double alpha, double beta);


double LRL_first_distance(double alpha, double beta, double d);
double LRL_middle_distance(double alpha, double beta, double d);
double LRL_last_distance(double alpha, double beta, double d);

double LRL_total_distance(double alpha, double beta, double d);
std::pair<double,double> LRL_possible_d(double alpha, double beta);


double SRS_first_distance(double alpha, double beta, double d);
double SRS_middle_distance(double alpha, double beta, double d);
double SRS_last_distance(double alpha, double beta, double d);

double SRS_total_distance(double alpha, double beta, double d);
std::pair<double,double> SRS_possible_d(double alpha, double beta);


double SLS_first_distance(double alpha, double beta, double d);
double SLS_middle_distance(double alpha, double beta, double d);
double SLS_last_distance(double alpha, double beta, double d);

double SLS_total_distance(double alpha, double beta, double d);
std::pair<double,double> SLS_possible_d(double alpha, double beta);


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

constexpr const std::array<std::string,3> DubinsMoveNames{
    std::string("STRAIGHT"),
    std::string("LEFT"),
    std::string("RIGHT")
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
void update_dubins(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius);

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
Pose3D follow_dubins(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius);

/********************  Recover path shape from two points and a hint  ********************/

/**
 * @brief Structure to hold parameters describing a simple path
 * 
 */
template<DubinsMove m>
struct PathShape
{
    double x,y,z;   // A reference point (typically starting point)
    double p1;  // For a Straight: horizontal x speed | For a circle: turn_radius
    double p2;  // For a Straight: horizontal y speed | For a circle: angular speed
    double p3;  // Vertical speed
    double p4;  // For a Straight: unused | For a circle: initial angle
};

/**
 * @brief Given a shape and two points on this shape, compute the parameters for its parametric equation
 * 
 * @tparam m Shape parameter, being on the Dubins move
 * @param start     First pose sampled
 * @param end       Second pose sampled
 * @param h_speed   XY vehicle speed
 * @param turn_radius   Turn radius for turns
 * @param v_speed   Z vehicle speed
 * @return PathShape    Computed parameters
 */
template<DubinsMove m>
PathShape<m> compute_params(const Pose3D& start, const Pose3D& end, double h_speed, double turn_radius, double v_speed);