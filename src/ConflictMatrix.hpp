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

#include <highs/Highs.h>

#include "Aircraft.h"
#include "Dubins.hpp"
#include "Primitives.hpp"

typedef std::vector<std::vector<bool>> aapp_feasability_matrix_T;
typedef std::vector<std::vector<std::unique_ptr<Dubins>>> ListOfPossibilities;

/**
 * @brief Map the strict upper triangular part of a distance matrix to a flat array.
 * 
 * If the matrix is of dimensions N x N, the array is of size N(N-1)/2 (only the strict upper triangle is stored).
 * The formula is as follow, assuming i < j < N :
 * 
 *  N * i + j - ((i + 2) * (i + 1)) // 2
 * 
 * (It amount to fetch the correct cell in a flattened matrix, but skipping the lower triangle each time)
 * 
 * @param i Row index, strictly less than N
 * @param j Column index, between i and j strictly
 * @param N Matrix dimension
 * @return uint Matching index in the flat array
 */
uint ac_index_map(uint i, uint j, uint N);

/**
 * @brief Map the rectangular matrix of dimensions P1 x P2 to a flat array in row-major fashion
 * 
 * The formula is as follow for indices i,j :
 * 
 *  P2 * i + j
 * 
 * @param i Row index
 * @param j Column index
 * @param P1 Number of rows
 * @param P2 Number of columns
 * @return uint Matchin index in the flat array
 */
uint path_index_map(uint i, uint j, uint P1, uint P2);

aapp_feasability_matrix_T compute_XY_separations(const ListOfPossibilities&, const std::vector<AircraftStats>&,double);

aapp_feasability_matrix_T parallel_compute_XY_separations(const ListOfPossibilities&, const std::vector<AircraftStats>&, double, 
    uint THREADS = std::thread::hardware_concurrency()/2);


std::vector<std::unique_ptr<Dubins>> find_pathplanning_LP_soltuion(const ListOfPossibilities&, const std::vector<AircraftStats>&, const aapp_feasability_matrix_T&);
