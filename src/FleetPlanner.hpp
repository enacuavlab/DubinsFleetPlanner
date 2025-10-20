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

#include <vector>
#include <tuple>
#include <memory>
#include <optional>
#include <format>

#include <boost/chrono.hpp>

#include "Dubins.hpp"
#include "BaseDubins.hpp"
#include "ExtendedDubins.hpp"

#include "ConflictList.hpp"

/********************************************************************************/
/********************************************************************************/

/**                                     Helpers                                 */

/********************************************************************************/
/********************************************************************************/

// ---------- Imports and typedefs ---------- //

namespace chrono = boost::chrono;

typedef std::optional<std::vector<std::unique_ptr<Dubins>>> UniqueDubinsResults;
typedef std::optional<std::vector<std::shared_ptr<Dubins>>> SharedDubinsResults;

// ---------- Metadata processing ---------- //

class ExtraPPResults
{
public:
    std::string case_name;
    bool success;
    uint iterations;
    chrono::nanoseconds duration;
    uint threads;
    uint possible_paths_num;
    double initial_path_time;
    double final_path_time;

    std::string format() const
    {
        return std::format(
            "Case: {}\n -> {}\n  Found path of duration {} (baseline is {})"
            "\n Performed {} iterations in {} ns\n Used {} threads; {} possible paths",
            case_name, 
            (success) ? "Success!" : "FAILURE",
            final_path_time,
            initial_path_time,
            iterations,
            duration.count(),
            threads,
            possible_paths_num);
    }


    static const std::string CSV_header()
    {
        return "Test input;Success;Iterations;Duration(ns);Threads;Possible paths;Initial guessed time;Final optained time";
    }


    std::string as_CSV() const
    {
        return std::format("{};{};{};{};{};{};{};{}",
            case_name,
            success,
            iterations,
            duration.count(),
            threads,
            possible_paths_num,
            initial_path_time,
            final_path_time
        );
    }
};

// ---------- Helpful for solving ---------- //

double maxmin_dubins_traveltime(
    const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
    const std::vector<AircraftStats>& stats, const std::vector<double>& delta_t,
    double wind_x, double wind_y
);

std::vector<double> compute_arrival_times(const std::vector<double>& dts, double time_ref);

std::tuple<uint,uint> count_min_number_of_valid_paths(const ListOfPossibilities& all_paths);


class AbstractFleetPlanner 
{
protected:

    /*******************************************************************************/
    /*                                                                             */
    /*                              Solver attributes                              */
    /*                                                                             */
    /*******************************************************************************/

    double precision_tol;               // Precision when looking for collisions
    double maximal_relative_duration;   // Maximum distance (as a multiplicative factor from the lowest possible distance)
    mutable Highs ref_model;                    // Reference HiGHS model for fast init


    /*******************************************************************************/
    /*                                                                             */
    /*                               Path generation                               */
    /*                                                                             */
    /*******************************************************************************/

    /**
     * @brief Virtual function generating possible paths given a situation
     * 
     * @param start     Starting pose
     * @param end       Ending pose 
     * @param stats     Aircraft statistics
     * @param target_len    Length of the paths to generate
     * @return std::vector<std::unique_ptr<Dubins>> List of possible paths with the given length
     */
    virtual std::vector<std::unique_ptr<Dubins>> generate_possible_paths(
        const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const = 0;

    /**
     * @brief Maximum number of different paths returned by `generate_possible_paths` 
     * 
     * @return uint 
     */
    virtual uint max_path_num() const = 0; 


    /**********************************************************************************/
    /*                                                                                */
    /*          Find a collision-free solution given a list of possibilities          */
    /*                                                                                */
    /**********************************************************************************/

    /******************** Mono-thread solution finding ********************/

    /**
     * @brief Recursively look for a valid combination of paths
     * 
     * This is akin to a depth-first search in the tree of possible combinations, with 'naive' branch-and-bound (i.e. there no special heuristic
     * for selecting better branches)
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param durations Trip duration for each aircraft
     * @param min_sep   Minimal distance required
     * @param current_index Index of the aircraft currently being studied
     * @param choices   Array of selected paths; considered filled for i=0 to current_index (excluded)
     * @param conflicts_memo Hashmap storing known conflict values
     * @return true A valid combination has been found, as is stored in 'choices'
     * @return false No valid combination found
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    bool recursive_pathfinder(const ListOfPossibilities& all_paths, const std::vector<AircraftStats>& stats,
        double min_sep, uint current_index, 
        std::vector<int>& choices, std::map<Conflict_T,bool>& conflicts_memo) const
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
                    Conflict_T ccase = std::make_tuple(
                        current_index,i,
                        past_index,choices[past_index]);

                    
                    auto case_it = conflicts_memo.find(ccase);
                    if (case_it == conflicts_memo.end()) // If not found, compute and store
                    {
                        double duration = std::min(
                            dubin->get_length()/this_speed,
                            other->get_length()/other_speed);

                        conflict_with_existing = !separation_function(*dubin,*other,this_speed,other_speed,duration,min_sep,precision_tol);
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
                    bool successful_attribution = recursive_pathfinder<separation_function>(
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

    /**
     * @brief Recursively look for a valid combination of paths
     * 
     * This is an entry point for the 'recursive_pathfinder' method
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param min_sep   Minimal distance required
     * @return UniqueDubinsResults
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    UniqueDubinsResults pathfinder_separated(
        ListOfPossibilities& all_paths, const std::vector<AircraftStats>& stats, double min_sep) const
    {
        uint current_index = 0;
        std::vector<int> choices(stats.size());
        std::map<Conflict_T,bool> conflicts_memo;

        bool success = recursive_pathfinder<separation_function>(all_paths,stats,min_sep,current_index,choices,conflicts_memo);
        if (success)
        {
            std::vector<std::unique_ptr<Dubins>> output(all_paths.size());

            for(uint i = 0; i < all_paths.size(); i++)
            {
                output[i] = std::move(all_paths[i][choices[i]]);
            }

            return output;
        }
        else
        {
            return std::nullopt;
        }
    }

    /******************** Multi-thread solution finding ********************/




    /******************** Standardized entry points ********************/


    /**
     * @brief Generic function for monothread solution finding (may be overloaded)
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param min_sep   Minimal distance required
     * @return UniqueDubinsResults
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    UniqueDubinsResults find_solution(
        ListOfPossibilities& possibilites, const std::vector<AircraftStats>& stats, double min_sep) const
    {
        return pathfinder_separated<separation_function>(possibilites,stats,min_sep);
    }

    /**
     * @brief Generic function for multithread solution finding (may be overloaded)
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param min_sep   Minimal distance required
     * @param threads   Number of threads to use
     * @return SharedDubinsResults
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    SharedDubinsResults find_solution_parallel(
        ListOfPossibilities& possibilites, const std::vector<AircraftStats>& stats, double min_sep, uint threads=0) const
    {
        SharedListOfPossibilities shared_list;
        for(auto& vd: possibilites)
        {
            shared_list.push_back(make_shared(vd));
        }

        std::vector<Conflict_T> conflicts = parallel_compute_XY_separations(shared_list,stats,min_sep,threads);

        return find_pathplanning_LP_solution(shared_list,stats,conflicts,max_path_num(),threads);
    }

    ListOfPossibilities list_all_possibilities(
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, const std::vector<double>& times, double wind_x, double wind_y) const
    {
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == times.size());
#endif
        uint N = starts.size();

        ListOfPossibilities output(N);

        for(uint i = 0; i < N; i++)
        {
            // Modify target according to estimated travel time
            Pose3D base_end = ends[i];
            double target_time = times[i];
            base_end.x -= target_time*wind_x;
            base_end.y -= target_time*wind_y;
            double target_len = target_time*stats[i].airspeed;

            // List all possibilities for aircraft i
            output[i] = generate_possible_paths(starts[i],base_end,stats[i],target_len);
        }

        return output;
    }


public:
    int verbosity;
    AbstractFleetPlanner(double _prec, double _max_r_dur, int verb) : 
        precision_tol(_prec), maximal_relative_duration(_max_r_dur), verbosity(verb) {}

    AbstractFleetPlanner(double _prec, double _max_r_dur) :
        precision_tol(_prec), maximal_relative_duration(_max_r_dur), verbosity(1) {}

    /**
     * @brief Solve the pathfinding problem using a monothread solver
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param extra     Extra information regarding the solving process
     * @param starts    List of starting poses 
     * @param ends      List of ending poses
     * @param stats     Statistics for aircraft
     * @param min_sep   Minimal required separation
     * @param delta_t   Wanted time separation between arrivals.
     * @param wind_x    X component of the wind vector. Default to no wind.
     * @param wind_y    Y component of the wind vector. Default to no wind.
     * @param max_iters Maximal number of iterations before aborting research. Default to 300
     * @return UniqueDubinsResults
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    UniqueDubinsResults solve(
        ExtraPPResults& extra,
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, double min_sep,
        const std::vector<double>& delta_t,
        double wind_x = 0., double wind_y = 0.,
        uint max_iters = 300
    ) const
    {

        uint N = starts.size();
        extra.threads = 0;
        extra.possible_paths_num = max_path_num();
         
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == delta_t.size() +1);

        for(uint i = 0; i < N; i++)
        {
            for(uint j = i+1; j < N; j++)
            {
                assert(pose_dist_XY(starts[i],starts[j]) > min_sep);
            }
        }
#endif

        chrono::process_real_cpu_clock clk;
        auto start = clk.now();

        min_sep = std::abs(min_sep);

        double target_time = maxmin_dubins_traveltime(starts,ends,stats,delta_t,wind_x,wind_y);

        extra.initial_path_time = target_time;

        double max_time = target_time * maximal_relative_duration;

        double time_step = max_time/max_iters;

        uint current_iter = 0;

        while(current_iter < max_iters)
        {

            if (verbosity > 0)
            {
                std::cout   << "Starting iteration number " << current_iter 
                            << " (target time: " << target_time << " )" << std::endl;
            }

            std::vector<double> times = compute_arrival_times(delta_t,target_time);

            ListOfPossibilities possibilities = list_all_possibilities(starts,ends,stats,times,wind_x,wind_y);

            uint min_possible_paths,min_loc;
            std::tie(min_possible_paths,min_loc) = count_min_number_of_valid_paths(possibilities);

            if (min_possible_paths > 0)
            {
                std::optional<std::vector<std::unique_ptr<Dubins>>> sol = find_solution<separation_function>(possibilities,stats,min_sep);

                if (sol.has_value())
                {
                    auto end = clk.now();

                    extra.success       = true;
                    extra.duration      = end - start;
                    extra.iterations    = current_iter;
                    extra.final_path_time = target_time;
                    return sol;
                }
            }
            else if (verbosity > 0)
            {
                std::cout   << "AC number " << min_loc << " has no path available..." << std::endl;
            }

            current_iter++;
            target_time += time_step;
        }
        
        auto end = clk.now();

        extra.success       = false;
        extra.duration      = end - start;
        extra.iterations    = current_iter;
        extra.final_path_time = target_time;
        return std::nullopt;
    }

    /**
     * @brief Solve the pathfinding problem using a multithread solver
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @param extra     Extra info regarding the solving process
     * @param starts    List of starting poses 
     * @param ends      List of ending poses
     * @param stats     Statistics for aircraft
     * @param min_sep   Minimal required separation
     * @param delta_t   Wanted time separation between arrivals.
     * @param wind_x    X component of the wind vector. Default to no wind.
     * @param wind_y    Y component of the wind vector. Default to no wind.
     * @param max_iters Maximal number of iterations before aborting research. Default to 300
     * @param threads   Number of threads to use for solving. Default to half of `std::thread::hardware_concurrency()`
     * @return SharedDubinsResults
     */
    template<Dubins::DubinsSeparationFunction separation_function>
    SharedDubinsResults solve_parallel(
        ExtraPPResults& extra,
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, double min_sep,
        const std::vector<double>& delta_t,
        double wind_x = 0., double wind_y = 0.,
        uint max_iters = 300, uint threads = std::thread::hardware_concurrency()/2
    ) const
    {
        #if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == delta_t.size() +1);
        #endif
        
        if (threads == 0)
        {
            threads = std::thread::hardware_concurrency();
        }

        extra.threads = threads;
        min_sep = std::abs(min_sep);
        extra.possible_paths_num = max_path_num();

        chrono::process_real_cpu_clock clk;
        auto start = clk.now();

        setup_base_model(ref_model,starts.size(),max_path_num(),verbosity);

        double target_time = maxmin_dubins_traveltime(starts,ends,stats,delta_t,wind_x,wind_y);

        extra.initial_path_time = target_time;

        double max_time = target_time * maximal_relative_duration;

        double time_step = max_time/max_iters;

        uint current_iter = 0;

        while(current_iter < max_iters)
        {

            if (verbosity > 0)
            {
                std::cout   << "Starting iteration number " << current_iter 
                            << " (target time: " << target_time << " )" << std::endl;
            }

            std::vector<double> times = compute_arrival_times(delta_t,target_time);

            ListOfPossibilities possibilities = list_all_possibilities(starts,ends,stats,times,wind_x,wind_y);

            std::optional<std::vector<std::shared_ptr<Dubins>>> sol = find_solution_parallel<separation_function>(possibilities,stats,min_sep,threads);

            if (sol.has_value())
            {
                auto end = clk.now();

                extra.success       = true;
                extra.duration      = end - start;
                extra.iterations    = current_iter;
                extra.final_path_time = target_time;
                return sol;
            }

            current_iter++;
            target_time += time_step;
        }

        auto end = clk.now();

        extra.success       = false;
        extra.duration      = end - start;
        extra.iterations    = current_iter;
        extra.final_path_time = target_time;
        return std::nullopt;
    }
};


/********************************************************************************/
/***                                                                          ***/
/***                 Class implementation for path generation                 ***/
/***                                                                          ***/
/********************************************************************************/

/**
 * @brief Plannification based only on the 6+2 basic Dubins paths
 * 
 */
class BasicDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return fit_possible_baseDubins(stats.climb,stats.turn_radius,start,end,target_len,precision_tol);
    }

    uint max_path_num() const
    {
        return NumberOfBaseDubins;
    }

public:
    BasicDubinsFleetPlanner(double _prec, double _max_r_dur) :
        AbstractFleetPlanner(_prec,_max_r_dur) {}

    BasicDubinsFleetPlanner(double _prec, double _max_r_dur, int verb) :
        AbstractFleetPlanner(_prec,_max_r_dur,verb) {}
};

/**
 * @brief Plannification based on the 8 basic Dubins paths extended by basic primitives
 * 
 */
class ExtendedDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<double> start_lengths,end_lengths; // Finite list of possible lengths for starting and ending extra path

    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return generate_all_extended_from_fitted_baseDubins(start,end,start_lengths,end_lengths,stats.climb,stats.turn_radius,target_len,precision_tol);
    }

    uint max_path_num() const
    {
        bool zero_in_starts = false;
        bool zero_in_ends   = false;

        for(double val : start_lengths)
        {
            if (abs(val) < precision_tol)
            {
                zero_in_starts = true;
                break;
            }
        }

        for(double val : end_lengths)
        {
            if (abs(val) < precision_tol)
            {
                zero_in_ends = true;
                break;
            }
        }

        uint nonzero_starts = start_lengths.size()  + ((zero_in_starts) ? (-1) : 0);
        uint nonzero_ends   = end_lengths.size()    + ((zero_in_ends)   ? (-1) : 0);

        uint nbr_of_possibilities = DubinsMoveNum*nonzero_starts*NumberOfBaseDubins*nonzero_ends*DubinsMoveNum; // Non-zero paths

        if (zero_in_starts)
        {
            nbr_of_possibilities += NumberOfBaseDubins*nonzero_ends*DubinsMoveNum; // Starts with 0, ends with non-zero
        }

        if (zero_in_ends)
        {
            nbr_of_possibilities += DubinsMoveNum*nonzero_starts*NumberOfBaseDubins; // Ends with 0, starts with non-zero
        }

        if (zero_in_starts && zero_in_ends)
        {
            nbr_of_possibilities += DubinsMoveNum; // Starts and ends with 0
        }

        return nbr_of_possibilities;
    } 

public:
    ExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens)
        : AbstractFleetPlanner(_prec,_max_r_dur), start_lengths(_slens), end_lengths(_elens) {}

    ExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb), start_lengths(_slens), end_lengths(_elens) {}

};