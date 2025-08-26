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

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
#include <iostream>
#endif

#include <array>
#include <map>
#include <optional>

#include "Aircraft.h"
#include "Dubins.hpp"
#include "BaseDubins.hpp"

namespace DubinsPP
{

    namespace BasicDubins
    {
        /**
         * @brief Compute the individual shortest paths for all dubins, ignoring collisions
         * 
         * @tparam N Number of aircraft
         * @param starts    Starting points
         * @param ends      Ending points, ordered by rank of arrival
         * @param stats     Aircraft characteristics (speed, climb rate, turn radius)
         * @param wind_x    Wind, x component (default to 0 if unspecified)
         * @param wind_y    Wind, y component (default to 0 if unspecified)
         * @return std::array<std::unique_ptr<Dubins>,N> 
         */
        template<uint N>
        std::array<std::unique_ptr<Dubins>,N> shortest_dubins(
            const std::array<Pose3D,N>& starts, const std::array<Pose3D,N>& ends, const std::array<AircraftStats,N>& stats,
            double wind_x = 0., double wind_y = 0.
        );

        /**
         * @brief Compute the individual shortest paths for all dubins, ignoring collisions
         * 
         * @param starts    Starting points
         * @param ends      Ending points, ordered by rank of arrival
         * @param stats     Aircraft characteristics (speed, climb rate, turn radius)
         * @param wind_x    Wind, x component (default to 0 if unspecified)
         * @param wind_y    Wind, y component (default to 0 if unspecified)
         * @return std::array<std::unique_ptr<Dubins>,N> 
         */
        std::vector<std::unique_ptr<Dubins>> shortest_dubins(
            const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends, const std::vector<AircraftStats>& stats,
            double wind_x = 0., double wind_y = 0.
        );

        /**
         * @brief Compute a synchronized path planning for a team of fixed-wing aircraft, ignoring collisions
         * 
         * Proceed by estimating a minimal travel time (using individual path planning), then try to compute a solution
         * with the given travel time estimation. If it works, return the solution. Otherwise, increase the travel time and retry.
         * The travel time is linearly increased from the minimum estimate t_min to t_min * max_r_duration with max_iters samples.
         * 
         * This version uses only basic Dubins paths for its planning.
         * 
         * @tparam N Number of aircraft
         * @param starts    Starting points
         * @param ends      Ending points, ordered by rank of arrival
         * @param stats     Aircraft characteristics (speed, climb rate, turn radius)
         * @param delta_t   By how much time the arrival of aircraft N+1 should be delayed from aircraft N. Default to 0s (synchronised arrivals)
         * @param wind_x    Wind, x component (default to 0 if unspecified)
         * @param wind_y    Wind, y component (default to 0 if unspecified)
         * @param max_r_duration Maximum relative duration, ie by how much the initial travel time guess can be multiplied by. Must be greater than 1. Default to 3.
         * @param t_tol     Tolerance (absolue) for timing accuracy. Must be positive. Default to 1e-6
         * @param max_iters Maximum number of iterations. Must be greater than 2. Default to 100
         * @return std::array<std::unique_ptr<Dubins>,N> 
         */
        template<uint N>
        std::optional<std::array<std::unique_ptr<Dubins>,N>> synchronised_no_checks(
            const std::array<Pose3D,N>& starts, const std::array<Pose3D,N>& ends, const std::array<AircraftStats,N>& stats,
             const std::array<double,N-1>& delta_t = {0.}, double wind_x = 0., double wind_y = 0.,
            double max_r_duration = 3., double t_tol = 1e-6,
            uint max_iters = 100
        );

        /**
         * @brief Compute a synchronized path planning for a team of fixed-wing aircraft, ignoring collisions
         * 
         * Proceed by estimating a minimal travel time (using individual path planning), then try to compute a solution
         * with the given travel time estimation. If it works, return the solution. Otherwise, increase the travel time and retry.
         * The travel time is linearly increased from the minimum estimate t_min to t_min * max_r_duration with max_iters samples.
         * 
         * This version uses only basic Dubins paths for its planning.
         * 
         * @param starts    Starting points
         * @param ends      Ending points, ordered by rank of arrival
         * @param stats     Aircraft characteristics (speed, climb rate, turn radius)
         * @param delta_t   By how much time the arrival of aircraft N+1 should be delayed from aircraft N.
         * @param wind_x    Wind, x component (default to 0 if unspecified)
         * @param wind_y    Wind, y component (default to 0 if unspecified)
         * @param max_r_duration Maximum relative duration, ie by how much the initial travel time guess can be multiplied by. Must be greater than 1. Default to 3.
         * @param t_tol     Tolerance (absolue) for timing accuracy. Must be positive. Default to 1e-6
         * @param max_iters Maximum number of iterations. Must be greater than 2. Default to 100
         * @return std::array<std::unique_ptr<Dubins>,N> 
         */
        std::optional<std::vector<std::unique_ptr<Dubins>>> synchronised_no_checks(
            const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends, const std::vector<AircraftStats>& stats,
            const std::vector<double>& delta_t,
            double wind_x = 0., double wind_y = 0.,
            double max_r_duration = 3., double t_tol = 1e-6,
            uint max_iters = 100
        );

        /**
         * @brief Compute a synchronized path planning for a team of fixed-wing aircraft, considering XY separation
         * 
         * Proceed by estimating a minimal travel time (using individual path planning), then try to compute a solution
         * with the given travel time estimation. If it works, return the solution. Otherwise, increase the travel time and retry.
         * The travel time is linearly increased from the minimum estimate t_min to t_min * max_r_duration with max_iters samples.
         * 
         * This version uses only basic Dubins paths for its planning.
         * 
         * @tparam N Number of aircraft
         * @param starts    Starting points
         * @param ends      Ending points, ordered by rank of arrival
         * @param stats     Aircraft characteristics (speed, climb rate, turn radius)
         * @param min_sep   Minimal XY euclidean distance that must be kept between two aircraft at all time
         * @param wind_x    Wind, x component (default to 0 if unspecified)
         * @param wind_y    Wind, y component (default to 0 if unspecified)
         * @param max_r_duration Maximum relative duration, ie by how much the initial travel time guess can be multiplied by. Must be greater than 1. Default to 3.
         * @param delta_t   By how much time the arrival of aircraft N+1 should be delayed from aircraft N. Default to 0s (synchronised arrivals)
         * @param t_tol     Tolerance (absolue) for timing accuracy. Must be positive. Default to 1e-6
         * @param max_iters Maximum number of iterations. Must be greater than 2. Default to 100
         * @return std::array<std::unique_ptr<Dubins>,N> 
         */
        template<uint N>
        std::optional<std::array<std::unique_ptr<Dubins>,N>> synchronised_XY_checks(
            const std::array<Pose3D,N>& starts, const std::array<Pose3D,N>& ends, const std::array<AircraftStats,N>& stats,
            double min_sep, const std::array<double,N-1>& delta_t = {0.}, double wind_x = 0., double wind_y = 0.,
            double max_r_duration = 3., double t_tol = 1e-6,
            uint max_iters = 100
        );

        /**
         * @brief Compute a synchronized path planning for a team of fixed-wing aircraft, considering XY separation
         * 
         * Proceed by estimating a minimal travel time (using individual path planning), then try to compute a solution
         * with the given travel time estimation. If it works, return the solution. Otherwise, increase the travel time and retry.
         * The travel time is linearly increased from the minimum estimate t_min to t_min * max_r_duration with max_iters samples.
         * 
         * This version uses only basic Dubins paths for its planning.
         * 
         * @param starts    Starting points
         * @param ends      Ending points, ordered by rank of arrival
         * @param stats     Aircraft characteristics (speed, climb rate, turn radius)
         * @param delta_t   By how much time the arrival of aircraft N+1 should be delayed from aircraft N. Default to 0s (synchronised arrivals)
         * @param min_sep   Minimal XY euclidean distance that must be kept between two aircraft at all time
         * @param wind_x    Wind, x component (default to 0 if unspecified)
         * @param wind_y    Wind, y component (default to 0 if unspecified)
         * @param max_r_duration Maximum relative duration, ie by how much the initial travel time guess can be multiplied by. Must be greater than 1. Default to 3.
         * @param t_tol     Tolerance (absolue) for timing accuracy. Must be positive. Default to 1e-6
         * @param max_iters Maximum number of iterations. Must be greater than 2. Default to 100
         * @return std::array<std::unique_ptr<Dubins>,N> 
         */
        std::optional<std::vector<std::unique_ptr<Dubins>>> synchronised_XY_checks(
            const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends, const std::vector<AircraftStats>& stats,
            const std::vector<double>& delta_t,
            double min_sep, double wind_x = 0., double wind_y = 0.,
            double max_r_duration = 3., double t_tol = 1e-6,
            uint max_iters = 100
        );


        
    }
}

/*******************************************************************/
/*                                                                 */
/*                    Templates implementations                    */
/*                                                                 */
/*******************************************************************/

// -------------------- Non-declared helper functions -------------------- //

namespace 
{
// Type to save if two given configurations are in conflict, identified by <AC id 1, traj type 1, AC id 2, traj type 2>
typedef std::tuple<uint,short uint,uint,short uint> conflict_case; 

// ---------- Util functions ---------- //

/**
 * @brief Given endpoints, characteristics and time differences for a team of aircraft, compute a lower bound on the reference travel time
 * (i.e. the one of the first aircraft)
 * 
 * We have that for each aircraft i, its travel time t_i must be at least larger than its individually minimal travel time o_i
 * Also, we have t_i = t_{i-1} + delta_t_{i-1}
 * Hence, if we take t_0 has a reference, we get:
 *  t_0 >= o_0
 *  For all i > 0, t_0 >= o_i - sum_{k=0}^{i-1} delta_t_{k}
 * 
 * @tparam endpoints_T  Container for endpoints (starts and ends)
 * @tparam stats_T      Container for aircraft characteristics
 * @tparam deltat_T     Container for arrival differences
 * @param starts Starting poses
 * @param ends  Ending poses
 * @param stats Characteristics
 * @param delta_t Time difference at arrival, such that t_i = t_{i-1} + delta_t_{i-1}
 * @return double Lower bound for the first aircraft travel time
 */
template<class endpoints_T, class stats_T, class deltat_T>
double abs_compute_max_of_mins_traveltime(const endpoints_T& starts, const endpoints_T& ends,
    const stats_T& stats, const deltat_T& delta_t)
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(starts.size() == ends.size());
    assert(starts.size() == stats.size());
    assert(starts.size()-1 <= delta_t.size());
#endif

    uint N = starts.size();

    double min_travel_time;
    double delta_sum = 0.;
    for(uint i = 0; i < N; i++)
    {
        const Pose3D& s        = starts[i];
        const Pose3D& e        = ends[i];
        const AircraftStats& p = stats[i];

        std::unique_ptr<Dubins> dd = shortest_possible_baseDubins(
            p.climb,
            p.turn_radius,
            s,e
        );

        if (i == 0)
        {
            min_travel_time = dd->get_length()/p.airspeed;
        }
        else
        {
            min_travel_time = std::max(
                min_travel_time,
                dd->get_length()/p.airspeed - delta_sum);
            
            delta_sum += delta_t[i-1];
        }
        
    }

    return min_travel_time;
}

template<uint N>
inline double compute_max_of_mins_traveltime(const std::array<Pose3D,N>& starts, const std::array<Pose3D,N>& ends,
    const std::array<AircraftStats,N>& stats, const std::array<double,N-1>& delta_t)
{
    return abs_compute_max_of_mins_traveltime(starts,ends,stats,delta_t);
}

double compute_max_of_mins_traveltime_vec(const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
    const std::vector<AircraftStats>& stats, const std::vector<double>& delta_t)
{
    return abs_compute_max_of_mins_traveltime(starts,ends,stats,delta_t);
}



// ---------- XY Separated helpers ---------- //

/**
 * @brief Recursively look for a valid combination of paths, conflicts being considered only on a XY basis
 * 
 * This is akin to a depth-first search in the tree of possible combinations, with 'naive' branch-and-bound (i.e. there no special heuristic
 * for selecting better branches)
 * 
 * @tparam allpaths_T   Container for all possible paths
 * @tparam stats_T      Container for aircraft characteristics
 * @tparam choices_T    Container for currently selected paths
 * @param all_paths List of all possible path for each agent
 * @param stats     Statistics of each agent
 * @param durations Trip duration for each aircraft
 * @param min_sep   Minimal XY euclidean distance required
 * @param current_index Index of the aircraft currently being studied
 * @param choices   Array of selected paths; considered filled for i=0 to current_index (excluded)
 * @param conflicts_memo Hashmap storing known conflict values
 * @return true A valid combination has been found, as is stored in 'choices'
 * @return false No valid combination found
 */
template<class allpaths_T, class stats_T, class choices_T>
bool recursive_pathfinder_XY(const allpaths_T& all_paths, const stats_T& stats,
    double min_sep, uint current_index, 
    choices_T& choices, std::map<conflict_case,bool>& conflicts_memo)
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(all_paths.size() == choices.size());
    assert(all_paths.size() == stats.size());
#endif
    uint N = all_paths.size();

    // Edge case handling
    if (current_index >= N)
    {
        return true;
    }

    double this_speed = stats[current_index].airspeed;

    for(uint i = 0; i < all_paths[current_index].size(); i++)
    {
        // Take a possible path
        auto& dubin = all_paths[current_index][i];
        
        // if it is not valid, skip it
        if (!dubin->is_valid())
        {
            continue;
        }
        else // Otherwise, test distances against the ones already set
        {
            bool conflict_with_existing = false;

            for(uint past_index = 0; past_index < current_index; past_index++)
            {
                // Check if there is a conflict with a set path
                auto& other = all_paths[past_index][choices[past_index]];
                double other_speed = stats[past_index].airspeed;

                // Look first in the memo dictionnary
                conflict_case ccase = std::make_tuple(current_index,i,past_index,choices[past_index]);
                auto case_it = conflicts_memo.find(ccase);
                if (case_it == conflicts_memo.end()) // If not found, compute and store
                {
                    double duration = std::min(
                        dubin->get_length()/this_speed,
                        other->get_length()/other_speed);

                    conflict_with_existing = !dubin->is_XY_separated_from(*other,this_speed,other_speed,duration,min_sep);
                    conflicts_memo.insert({ccase,conflict_with_existing});
                }
                else    // Otherwise, simply retrieve the value
                {
                    conflict_with_existing = case_it->second;
                }

                // If there is, stop trying with this path
                if (conflict_with_existing)
                {
                    break;
                }
            }

            // Found a conflict between set paths and current; try the next one
            if (conflict_with_existing)
            {
                continue;
            }
            else // If not, set it and go down in recursion
            {
                choices[current_index] = i;
                bool successful_attribution = recursive_pathfinder_XY<allpaths_T,stats_T,choices_T>(
                    all_paths,stats,min_sep,current_index+1,choices,conflicts_memo);

                // Early return
                if (successful_attribution)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

template<class allpaths_T, class stats_T, class choices_T>
std::optional<choices_T> pathfinder_XY_separation(const allpaths_T& all_paths, const stats_T& stats, double min_sep)
{
    uint current_index = 0;
    choices_T choices;
    std::map<conflict_case,bool> conflicts_memo;

    bool succes = recursive_pathfinder_XY<allpaths_T,stats_T,choices_T>(all_paths,stats,min_sep,current_index,choices,conflicts_memo);
    if (succes)
    {
        return choices;
    }
    else
    {
        return std::nullopt;
    }

}


}


// ---------- No checks functions ---------- //

// template<uint N>
// std::array<std::unique_ptr<Dubins>,N> shortest_dubins(
//     const std::array<Pose3D,N>& starts, const std::array<Pose3D,N>& ends, const std::array<AircraftStats,N>& stats,
//     double wind_x = 0., double wind_y = 0.)
// {
//     for(uint i = 0; i < N; i++)
//     {
//         const Pose3D& s        = starts[i];
//         const Pose3D& e        = ends[i];
//         const AircraftStats& p = stats[i];

//         std::unique_ptr<Dubins> dd = shortest_possible_baseDubins(
//             p.climb,
//             p.turn_radius,
//             s,e
//         );

//         if (i == 0)
//         {
//             min_travel_time = dd->get_length()/p.airspeed;
//         }
//         else
//         {
//             min_travel_time = std::max(
//                 min_travel_time,
//                 dd->get_length()/p.airspeed - delta_sum);
            
//             delta_sum += delta_t[i-1];
//         }
        
//     }
// }


// std::vector<std::unique_ptr<Dubins>> shortest_dubins(
//     const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends, const std::vector<AircraftStats>& stats,
//     double wind_x = 0., double wind_y = 0.)
// {

// }

template<uint N>
std::optional<std::array<std::unique_ptr<Dubins>,N>> DubinsPP::BasicDubins::synchronised_no_checks(
    const std::array<Pose3D,N>& starts, const std::array<Pose3D,N>& ends, const std::array<AircraftStats,N>& stats,
    const std::array<double,N-1>& delta_t, double wind_x, double wind_y,
    double max_r_length, double t_tol,
    uint max_iters
)
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(max_r_length > 1.);
    assert(max_iters > 1);
    assert(t_tol > 0);
#endif
    double min_travel_time = compute_max_of_mins_traveltime<N>(starts,ends,stats,delta_t);

    uint iter_count = 0;
    double iter_step = min_travel_time*(max_r_length-1)/max_iters;

    std::array<std::vector<std::unique_ptr<Dubins>>,N> list_of_choices;
    while(iter_count < max_iters)
    {
        double travel_time = min_travel_time + iter_count*iter_step;

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
        std::cout   << "Starting iteration number " << iter_count 
                    << " (target time: " << travel_time << " )" << std::endl;
#endif
        double dt = 0.;
        bool next_iter = false;
        for(uint i = 0; i < N; i++)
        {
            // Modify target according to estimated travel time
            Pose3D base_end = ends[i];
            double target_time = (travel_time+dt);
            dt += delta_t[i];
            base_end.x -= target_time*wind_x;
            base_end.y -= target_time*wind_y;
            double target_len = target_time*stats[i].airspeed;

            // List all possibilities for aircraft i
            list_of_choices[i] = fit_possible_baseDubins(
                stats[i].climb,
                stats[i].turn_radius,
                starts[i],base_end,
                target_len,
                t_tol
            );

            // If nothing works, skip
            if (list_of_choices[i].size() == 0)
            {

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
                std::cout << "  Could not find solution for aircraft " << i << " ... Skipping" << std::endl << std::endl;
#endif
                next_iter = true;
                break;
            }
        }

        if (next_iter)
        {
            iter_count++;
            continue;
        }
        else
        {
            std::array<std::unique_ptr<Dubins>,N> output;
            for(uint i = 0; i < N; i++)
            {
                output[i] = std::move(list_of_choices[i][0]);
            }
            return output;
        }

    }
    
    return std::nullopt;
}

std::optional<std::vector<std::unique_ptr<Dubins>>> DubinsPP::BasicDubins::synchronised_no_checks(
    const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends, const std::vector<AircraftStats>& stats,
    const std::vector<double>& delta_t, double wind_x, double wind_y,
    double max_r_length, double t_tol,
    uint max_iters
)
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(max_r_length > 1.);
    assert(max_iters > 1);
    assert(t_tol > 0);
    assert(starts.size() == ends.size());
    assert(starts.size() == stats.size());
    assert(starts.size() == delta_t.size() -1);
#endif

    double min_travel_time = compute_max_of_mins_traveltime_vec(starts,ends,stats,delta_t);

    uint N = starts.size();

    uint iter_count = 0;
    double iter_step = min_travel_time*(max_r_length-1)/max_iters;

    std::vector<std::vector<std::unique_ptr<Dubins>>> list_of_choices;
    while(iter_count < max_iters)
    {
        list_of_choices.clear();
        double travel_time = min_travel_time + iter_count*iter_step;

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
        std::cout   << "Starting iteration number " << iter_count 
                    << " (target time: " << travel_time << " )" << std::endl;
#endif
        double dt = 0.;
        bool next_iter = false;
        for(uint i = 0; i < N; i++)
        {
            // Modify target according to estimated travel time
            Pose3D base_end = ends[i];
            double target_time = (travel_time+dt);
            dt += delta_t[i];
            base_end.x -= target_time*wind_x;
            base_end.y -= target_time*wind_y;
            double target_len = target_time*stats[i].airspeed;

            // List all possibilities for aircraft i
            list_of_choices.push_back(fit_possible_baseDubins(
                stats[i].climb,
                stats[i].turn_radius,
                starts[i],base_end,
                target_len,
                t_tol
            ));

            // If nothing works, skip
            if (list_of_choices[i].size() == 0)
            {

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
                std::cout << "  Could not find solution for aircraft " << i << " ... Skipping" << std::endl << std::endl;
#endif
                next_iter = true;
                break;
            }
        }

        if (next_iter)
        {
            iter_count++;
            continue;
        }
        else
        {
            std::vector<std::unique_ptr<Dubins>> output;
            for(uint i = 0; i < N; i++)
            {
                output.push_back(std::move(list_of_choices[i][0]));
            }
            return output;
        }

    }
    
    return std::nullopt;
    
}

// ---------- XY checks functions ---------- //


template<uint N>
std::optional<std::array<std::unique_ptr<Dubins>,N>> DubinsPP::BasicDubins::synchronised_XY_checks(
    const std::array<Pose3D,N>& starts, const std::array<Pose3D,N>& ends, const std::array<AircraftStats,N>& stats,
    double min_sep, const std::array<double,N-1>& delta_t, double wind_x, double wind_y,
    double max_r_length, double t_tol,
    uint max_iters
)
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(max_r_length > 1.);
    assert(max_iters > 1);
    assert(t_tol > 0);

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            assert(pose_dist_XY(starts[i],starts[j]) > min_sep);
        }
    }
#endif

    double min_travel_time = compute_max_of_mins_traveltime<N>(starts,ends,stats,delta_t);

    uint iter_count = 0;
    double iter_step = min_travel_time*(max_r_length-1)/max_iters;

    std::array<ArrayOfBaseDubins,N> list_of_choices;
    while(iter_count < max_iters)
    {
        double travel_time = min_travel_time + iter_count*iter_step;

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
        std::cout   << "Starting iteration number " << iter_count 
                    << " (target time: " << travel_time << " )" << std::endl;
#endif
        double dt = 0.;
        for(uint i = 0; i < N; i++)
        {
            // Modify target according to estimated travel time
            Pose3D base_end = ends[i];
            double target_time = (travel_time+dt);
            dt += delta_t[i];
            base_end.x -= target_time*wind_x;
            base_end.y -= target_time*wind_y;
            double target_len = target_time*stats[i].airspeed;

            // List all possibilities for aircraft i
            list_of_choices[i] = fit_all_baseDubins(
                stats[i].climb,
                stats[i].turn_radius,
                starts[i],base_end,
                target_len,
                t_tol
            );
        }

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
        std::cout   << "  - Fitting done, starting search" << std::endl; 
#endif

        std::optional<std::array<uint,N>> result = pathfinder_XY_separation
            <std::array<ArrayOfBaseDubins,N>,std::array<AircraftStats,N>,std::array<uint,N>>
            (list_of_choices,stats,min_sep);

        if (result.has_value())
        {

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
        std::cout   << "  - Solution found! Returning..." << std::endl << std::endl; 
#endif
            std::array<std::unique_ptr<Dubins>,N> output;
            std::array<uint,N> choices = result.value();

            for(uint i = 0; i < N; i++)
            {
                output[i] = std::move(list_of_choices[i][choices[i]]);
            }

            return output;
        }
        else
        {
#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
        std::cout   << "  - No solution..." << std::endl << std::endl; 
#endif
            iter_count++;
        }

    }

#if defined(DubinsFleetPlanner_DEBUG_MSG) && DubinsFleetPlanner_DEBUG_MSG > 0
        std::cout   << "-> No Solution found... Returning nullptrs" << std::endl << std::endl; 
#endif
    return std::nullopt;
}


