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

// ---------- Util functions ---------- //

/**
 * @brief Represent a conflict in the form (ac_id1,path_id1,ac_id2,path_id2)
 * 
 */
typedef std::tuple<unsigned short, unsigned short, unsigned short, unsigned short> Conflict_T;

/**
 * @brief Represent a conflict with its information in the form
 *  (ac_id1,path_id1,ac_id2,path_id2,location,value)
 * 
 */
typedef std::tuple<unsigned short, unsigned short, unsigned short, unsigned short, double, double> RichConflict_T;

/**
 * @brief Convert a vector of RichConflict_T into a vector of Conflict_T while dropping irrelevant conflicts (distance higher than given bound)
 * 
 * @param vec Vector of RichConflict_T
 * @param min_dist Minimal separation distance
 * @return std::vector<Conflict_T> 
 */
std::vector<Conflict_T> drop_conflict_details(const std::vector<RichConflict_T>& vec, double min_dist);

/**
 * @brief Log the characteristics of a conflict (location,distance)
 * 
 */
typedef std::map<Conflict_T,std::pair<double,double>> Conflict_Map_T;

typedef std::vector<std::vector<std::unique_ptr<Dubins>>> ListOfPossibilities;
typedef std::vector<std::vector<std::shared_ptr<Dubins>>> SharedListOfPossibilities;

uint number_of_valid_paths(const ListOfPossibilities& list);
size_t list_hash(const ListOfPossibilities& list);

// ---------- Separation functions ---------- //

// ----- Generic functions ----- //

/**
 * @brief Given a separation function, possible paths, aircraft characteristics and a minimal separation,
 * compute the list of conflicting paths
 * 
 * @tparam separation_function Given two paths with their aircarft stats and a minimum separation, returns if there is a conflict or not
 * @param list_of_possibilites  List of possible paths for each aircraft
 * @param stats                 Aircraft stats
 * @param sep                   Minimal separation required
 * @return std::vector<Conflict_T>  List of conflicting aircraft + paths combination
 */
template<Dubins::DubinsSeparationFunction separation_function>
std::vector<Conflict_T> generic_compute_separations(const ListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep);

/**
 * @brief Given a separation function, possible paths, aircraft characteristics and a minimal separation,
 * compute the list of conflicting paths using multiple threads
 * 
 * @tparam separation_function  Given two paths with their aircarft stats and a minimum separation, returns if there is a conflict or not
 * @param THREADS               Number of threads to use
 * @param list_of_possibilites  List of possible paths for each aircraft
 * @param stats                 Aircraft stats
 * @param sep                   Minimal separation required
 * @return std::vector<Conflict_T>  List of conflicting aircraft + paths combination
 */
template<Dubins::DubinsSeparationFunction separation_function>
std::vector<Conflict_T> generic_parallel_compute_separations(uint THREADS, const SharedListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep);

    
// ---------- Distance functions ---------- //

/**
 * @brief Given a distance function, possible paths, aircraft characteristics and a minimal separation,
 * compute the list of conflicting paths with the conflict location and evaluation
 * 
 * @tparam distance_function    Given two paths with their aircarft stats and a minimum separation, 
 *  returns the closest point separating them with associated distance
 * @param list_of_possibilites  List of possible paths for each aircraft
 * @param stats                 Aircraft stats
 * @param sep                   Minimal separation required
 * @param map                   A map containing hints for the solver trying to find conflicts
 * @param all_values            Whether to returns all evaluations or only the ones generating conflicts
 * @return std::vector<RichConflict_T>  List of conflicting aircraft + paths combination with the location and distance
 */
template<Dubins::DubinsDistanceFunction distance_function>
std::vector<RichConflict_T> generic_compute_distances(
    const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats,
    double sep, const Conflict_Map_T& map, bool all_values);

/**
 * @brief Given a distance function, possible paths, aircraft characteristics and a minimal separation,
 * compute the list of conflicting paths with the conflict location and evaluation using multiple threads
 * 
 * @tparam distance_function    Given two paths with their aircarft stats and a minimum separation, 
 *  returns the closest point separating them with associated distance
 * @param THREADS               Number of threads to use
 * @param list_of_possibilites  List of possible paths for each aircraft
 * @param stats                 Aircraft stats
 * @param sep                   Minimal separation required
 * @param map                   A map containing hints for the solver trying to find conflicts
 * @param all_values            Whether to returns all evaluations or only the ones generating conflicts
 * @return std::vector<RichConflict_T>  List of conflicting aircraft + paths combination with the location and distance
 */
template<Dubins::DubinsDistanceFunction distance_function>
std::vector<RichConflict_T> generic_parallel_compute_distances(
    uint THREADS, const SharedListOfPossibilities& list_of_possibilites, 
    const std::vector<AircraftStats>& stats, double sep,
    const Conflict_Map_T& map, bool all_values);


// TODO: Compute the full matrix of separations, in order to find the best global separation

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
 * @param verbosity Enable/Disable logging from HiGHS library (logging enabled for verbosity > 1)
 * @return Highs A preconfigured model
 */
void setup_base_model(Highs& highs, uint AC_count, uint max_paths_count, int verbosity=1);

/**
 * @brief Solve the path finding problem using HiGHS ILP solver
 * 
 * @param possibilites  List of possibles paths per aircraft
 * @param stats         Caracteristics of each aircraft
 * @param conflicts     List of conflicts (paths that cannot be taken together)
 * @param max_path_num  Maximum number of paths for an aircraft
 * @param THREADS       Number of threads to use. 0 choose automatically. Default to 0
 * @param preset_model  A preset HiGHS model to quickly setup
 * @return std::optional<std::vector<std::shared_ptr<Dubins>>> 
 */
std::optional<std::vector<std::shared_ptr<Dubins>>> find_pathplanning_LP_solution(
    SharedListOfPossibilities& possibilites, 
    const std::vector<AircraftStats>& stats, 
    const std::vector<Conflict_T>& conflicts,
    uint max_path_num, int THREADS = 0,
    const Highs* preset_model = nullptr);
