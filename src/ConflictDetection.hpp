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

#include <tuple>
#include <limits>
#include <Eigen/Dense>

#include "utils.hpp"
#include "Dubins.hpp"
#include "Primitives.hpp"

// ==================== Geometric distance ==================== //

/**
 * @brief Given two shapes, compute their XY geometric separation 
 * 
 * This amount to the minimal euclidean distance between the two shapes, *ignoring* the vertical component.
 * 
 * @tparam m1 First shape type (STRAIGHT or an turn, RIGHT or LEFT)
 * @tparam m2 Second shape type (STRAIGHT or an turn, RIGHT or LEFT)
 * @param s1 First shape parameters
 * @param s2 Second shape parameters
 * @param duration Duration (in s) for which the shape are followed, defining segment and circle arcs
 * @return double The minimal XY separation distance
 */
template<DubinsMove m1, DubinsMove m2>
double geometric_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);

/**
 * @brief Given two shapes, compute their vertical (Z) geometric separation
 * 
 * Since all shapes consider linear climbs, the exact type does not change a thing to the computations
 * 
 * @tparam m1 First shape type (STRAIGHT or an turn, RIGHT or LEFT)
 * @tparam m2 Second shape type (STRAIGHT or an turn, RIGHT or LEFT)
 * @param s1 First shape parameters
 * @param s2 Second shape parameters
 * @param duration Duration (in s) for which the shape are followed, defining the segment
 * @return double The minimal Z separation distance
 */
template<DubinsMove m1, DubinsMove m2>
double geometric_Z_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);

// ==================== Temporal distance ==================== //
//TODO 
template<DubinsMove m1, DubinsMove m2>
double temporal_3D_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);

template<DubinsMove m1, DubinsMove m2>
double temporal_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);

template<DubinsMove m1, DubinsMove m2>
double temporal_Z_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);
