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

#include "ProjectHeader.h"

#include <array>
#include <vector>
#include <thread>
#include <future>
#include <optional>

#include <highs/Highs.h>

#include "Aircraft.h"
#include "Dubins.hpp"
#include "Primitives.hpp"

struct Conflict_T
{
    uint ac_id1;
    uint ac_id2;
    uint path_id1;
    uint path_id2;
};

typedef std::vector<std::vector<std::unique_ptr<Dubins>>> ListOfPossibilities;
typedef std::vector<std::vector<std::shared_ptr<Dubins>>> SharedListOfPossibilities;

std::vector<Conflict_T> compute_XY_separations(const ListOfPossibilities&, const std::vector<AircraftStats>&,double);

std::vector<Conflict_T> parallel_compute_XY_separations(const SharedListOfPossibilities&, const std::vector<AircraftStats>&, double, 
    uint THREADS = std::thread::hardware_concurrency()/2);



/**
 * @brief Generate a base Highs model for our path finding problem
 * 
 * We denote N the number of aircraft and P the number of paths per aircraft.
 * The decisions variables are the x_{ik} with 0 <= i < N and 0 <= p < P describing
 * if aircraft i uses path p.
 * 
 * We have some general constraints:
 * - Variables are 01 (a path is either chosen or not):
 *     Forall i in [0,N-1], forall p in [0,P-1]  x_{ip} in {0,1}
 *     (i.e. x_{ip} is an integer with 0 <= x_{ip} <= 1)
 * - Each aircraft must be allocated to one and only one path:
 *     Forall i, sum_{p=0}^{P-1} x_{ip} = 1
 * 
 * We also have some specific constraints, defined using additional data:
 * - There a conflict matrix M with dimension N x N x P x P, such that M_{ijpq} is 1 if
 *   there is a conflict between path p of aircraft i and path q of aircraft j, and 0 otherwise.
 *   The associated 'collision-free' constraints are written as:
 *     Forall i,j and forall p,q M_{ijpq} * (x_{ip} + x_{jq}) <= 1
 *   (Note: the indices are flattened, assuming all aircraft have the same number of possible paths)
 * 
 * @param highs HiGHS instance to setup
 * @param AC_count Number of aircraft
 * @param max_paths_count Maximum number of possible paths per aircraft
 * @return Highs A preconfigured model
 */
void setup_base_model(Highs& highs, uint AC_count, uint max_paths_count);

std::optional<std::vector<std::shared_ptr<Dubins>>> find_pathplanning_LP_solution(
    SharedListOfPossibilities&, 
    const std::vector<AircraftStats>&, 
    const std::vector<Conflict_T>&,
    uint max_path_num,
    Highs* preset_model = nullptr);
