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


[[gnu::const]]
double LSL_total_distance(double alpha, double beta, double d);

[[gnu::const]]
double RSR_total_distance(double alpha, double beta, double d);

[[gnu::const]]
double RSL_total_distance(double alpha, double beta, double d);

[[gnu::const]]
double LSR_total_distance(double alpha, double beta, double d);

[[gnu::const]]
double RLR_total_distance(double alpha, double beta, double d);

[[gnu::const]]
double LRL_total_distance(double alpha, double beta, double d);

[[gnu::const]]
double SRS_total_distance(double alpha, double beta, double d);

[[gnu::const]]
double SLS_total_distance(double alpha, double beta, double d);