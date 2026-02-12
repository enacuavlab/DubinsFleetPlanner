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

#include <algorithm>
#include <vector>
#include <tuple>
#include <list>
#include <set>

#include <iostream>
#include <memory>
#include <optional>
#include <format>

#include <boost/chrono.hpp>

#include "Dubins.hpp"
#include "BaseDubins.hpp"
#include "ExtendedDubins.hpp"
#include "BaseExtendedDubins.hpp"

#include "ConflictList.hpp"
#include "ioUtils.hpp"

/*************************************************************************************/
/*************************************************************************************/

/**                                     Helpers                                     **/

/*************************************************************************************/
/*************************************************************************************/

// ---------- Imports and typedefs ---------- //

namespace chrono = boost::chrono;

typedef std::optional<std::vector<std::unique_ptr<Dubins>>> UniqueDubinsResults;
typedef std::optional<std::vector<std::shared_ptr<Dubins>>> SharedDubinsResults;

namespace oprint = DubinsPP::OutputPrinter;

// ---------- Metadata processing ---------- //

class ExtraPPResults
{
public:
    std::string case_name;
    bool success;
    bool false_positive = false;
    uint iterations;
    chrono::nanoseconds duration;
    uint threads;
    uint possible_paths_num;
    double initial_path_time;
    double final_path_time;
    double worst_improvement_rate;

    std::string format() const
    {
        return std::format(
            "Case: {}\n -> {}\n  Found path of duration {} (baseline is {})"
            "\n Performed {} iterations in {:.4f} ms\n Used {} threads; {} possible paths per AC; {} Worst rate measured (time unit per second of computation)",
            case_name, 
            (success) ? ((false_positive) ? "FALSE POSITIVE!" : "Success!") : "FAILURE",
            final_path_time,
            initial_path_time,
            iterations,
            static_cast<double>(duration.count())/1000000.,
            threads,
            possible_paths_num,
            worst_improvement_rate);
    }


    static const std::string CSV_header()
    {
        return "Test input;Success;False positive;Iterations;Duration(ns);Threads;Possible paths;Initial guessed time;Final obtained time;Worst rate(u/s)";
    }


    std::string as_CSV() const
    {
        return std::format("{};{};{};{};{};{};{};{};{};{}",
            case_name,
            success,
            false_positive,
            iterations,
            duration.count(),
            threads,
            possible_paths_num,
            initial_path_time,
            final_path_time,
            worst_improvement_rate
        );
    }
};

// ---------- Helpful for solving ---------- //

/**
 * @brief For each aircraft, compute the shortest path. Then return the maximal time of all minima. 
 * 
 * @param starts    Starting poses
 * @param ends      Ending poses
 * @param stats     Stats for each aircraft
 * @param delta_t   Arrival time differences
 * @param wind_x    Wind, x component
 * @param wind_y    Wind, y component
 * @return double   Max of min travel times
 */
double maxmin_dubins_traveltime(
    const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
    const std::vector<AircraftStats>& stats, const std::vector<double>& delta_t,
    double wind_x, double wind_y
);

std::vector<double> compute_arrival_times(const std::vector<double>& dts, double time_ref);

/**
 * @brief Return one of the aircraft with the least amount of possible paths
 * 
 * @param all_paths List of all paths per aircraft
 * @return std::tuple<uint,uint> (number,ac_id) for an aircraft with minimal amount of available paths
 */
std::tuple<uint,uint> count_min_number_of_valid_paths(const ListOfPossibilities& all_paths);

/**
 * @brief Return the set of aircraft which do not have a possible path
 * 
 * @param all_paths List of all paths per aircraft
 * @return std::set<uint> set of ac_id, for which every individual has no path available
 */
std::set<uint> acs_without_paths(const ListOfPossibilities& all_paths);

/**
 * @brief Remove from the given list of paths the ones colliding with the given set of obstacle paths
 * 
 * @tparam distance_function    Function computing the distance between two path (usually underline the separation one)
 * @param all_paths             Candidate paths to be filtered
 * @param stats                 Aircraft stats associated to candidate paths
 * @param obstacle_paths          Obstacle paths
 * @param obstacle_stats        Stats of obstacle aircraft paths
 */
template<Dubins::DubinsDistanceFunction distance_function>
void filter_colliding_paths(ListOfPossibilities& all_paths, const std::vector<AircraftStats> &stats,
    const std::vector<std::shared_ptr<Dubins>>& obstacle_paths, const std::vector<AircraftStats>& obstacle_stats,
    double min_sep)
{
    for(uint i = 0; i < stats.size(); i++)
    {
        const AircraftStats& s = stats[i];
        std::vector<std::unique_ptr<Dubins>>& path_list = all_paths[i];
        std::vector<std::unique_ptr<Dubins>> filtered_paths;
        
        for(std::unique_ptr<Dubins>& p_ptr : path_list)
        {
            bool conflict_free = true;

            for(uint j = 0; j < obstacle_stats.size(); j++)
            {
                const Dubins& p = *p_ptr;
                const Dubins& c = *obstacle_paths[j];
                const AircraftStats& cs = obstacle_stats[j];
                 

                std::pair<double,double> res = generic_compute_distance<distance_function>(p,c,s,cs,min_sep);

                if (res.second < min_sep)
                {
                    conflict_free = false;
                }
            }

            if (conflict_free)
            {
                filtered_paths.push_back(std::move(p_ptr));
            }
        }

        all_paths[i] = std::move(filtered_paths);
    }
}

// ---------- Smart sampling ---------- //

struct SampleCase
{
    double val;
    bool done       = false;
    bool success    = false;
    std::set<uint> NoPathsACs;
};

void print_samples(const std::list<SampleCase>& l)
{
    for(const SampleCase& c : l)
    {
        std::cout << c.val << " ; ";
    }
    std::cout << std::endl;
}

std::list<SampleCase> generate_linspace(double start, double end, uint samples, bool include_start = true, bool include_end=true)
{
    assert(end > start);
    std::list<SampleCase> output;

    double delta = end-start;

    if (include_start && include_end)
    {
        assert(samples >= 2);

        double step = delta/(samples-1);
        for(uint i = 0; i < samples; i++)
        {
            output.push_back(SampleCase{start+i*step});
        }
        return output;
    }
    else if (include_start && !include_end)
    {
        assert(samples >= 1);

        double step = delta/(samples);
        for(uint i = 0; i < samples; i++)
        {
            output.push_back(SampleCase{start+i*step});
        }
        return output;
    }
    else if (!include_start && include_end)
    {
        assert(samples >= 1);

        double step = delta/(samples);
        for(uint i = 1; i <= samples; i++)
        {
            output.push_back(SampleCase{start+i*step});
        }
        return output;
    }
    else // (!include_start && !include_end)
    {
        double step = delta/(samples+1);
        for(uint i = 1; i <= samples; i++)
        {
            output.push_back(SampleCase{start+i*step});
        }
        return output;
    }
}

/**
 * @brief Assuming the input is sorted (by val), weave points, that is add points between the existing ones. 
 * Returns whether points have been successfully added or not.
 * 
 * @param q     List of SampleCase, sorted by val
 * @param weave Number of points to add between each two samples
 * @param min_weave_delta   Minimal value between two iterations for weaving between them.
 * @return true     New points have been added
 * @return false    No new samples
 */
bool weave_samples(std::list<SampleCase>& l, uint weave, double min_weave_delta)
{
    assert(l.size() > 1);
    // print_samples(l);

    auto next = l.begin(); next++;
    auto iter = l.begin();
    bool output = false;

    while(next != l.end())
    {
        if (next->val - iter->val >= min_weave_delta)
        {
            std::set<uint> intersect;
            std::set_intersection(
                iter->NoPathsACs.begin(),iter->NoPathsACs.end(),
                next->NoPathsACs.begin(),next->NoPathsACs.end(),
                std::inserter(intersect,intersect.begin())
            );

            if (intersect.size() == 0)
            {
                auto new_values = generate_linspace(iter->val,next->val,weave,false,false);
                l.splice(next,new_values);
                output = true;
            }
        }
        iter = next;
        next++;
    }

    return output;
}

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
    mutable Highs* ref_model = nullptr;            // Reference HiGHS model for fast init
    mutable bool first_log = true;      // When logging, state whether it is the first log or not

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

                        conflict_with_existing = !separation_function(*dubin,*other,this_speed,other_speed,duration,min_sep,precision_tol,DubinsDistDefaultRec);
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
     * @tparam distance_function    Function computing the distance between two path (usually underline the separation one)
     * @param all_paths List of all possible path for each agent
     * @param stats     Statistics of each agent
     * @param min_sep   Minimal distance required
     * @param threads   Number of threads to use
     * @return SharedDubinsResults
     */
    template<Dubins::DubinsSeparationFunction separation_function, Dubins::DubinsDistanceFunction distance_function>
    SharedDubinsResults find_solution_parallel(
        ListOfPossibilities& possibilites, const std::vector<AircraftStats>& stats, double min_sep, uint max_pathnum, uint threads=0,
        bool list_conflicts=true, double timetag=0.) const
    {
        SharedListOfPossibilities shared_list;
        for(auto& vd: possibilites)
        {
            shared_list.push_back(make_shared(vd));
        }
        
        std::vector<Conflict_T> conflicts;

        // If the minimum separation is too small, assume it is null and return no conflicts
        if (min_sep < DubinsFleetPlanner_PRECISION) 
        {
            return find_pathplanning_LP_solution(shared_list,stats,conflicts,max_pathnum,threads,ref_model);
        } 

        if (list_conflicts)
        {
            std::vector<RichConflict_T> r_conflicts = generic_parallel_compute_distances<distance_function>(
                threads,shared_list,stats,min_sep,
                Conflict_Map_T(),true);

            if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
            {
                oprint::append_rich_conflicts(datalog,timetag,r_conflicts,shared_list,stats,!first_log);
                first_log = false;
            }
            
            // std::cout << "Going from " << r_conflicts.size() << " rich conflicts to ";


            conflicts = drop_conflict_details(r_conflicts,min_sep);

            // std::cout << conflicts.size() << " std ones " << std::endl;

            
        }
        else
        {
            conflicts = generic_parallel_compute_separations<separation_function>(threads,shared_list,stats,min_sep);
        }

        return find_pathplanning_LP_solution(shared_list,stats,conflicts,max_pathnum,threads,ref_model);
    }

    /**
     * @brief List all possible paths for a fleet given their arrival times
     * 
     * @param starts    Starting poses
     * @param ends      Ending poses (ground referential)
     * @param stats     Aircraft statistics
     * @param times     Time of arrival for each aircraft
     * @param wind_x    Wind, X component
     * @param wind_y    Wind, Y component
     * @return ListOfPossibilities List of possible paths per aircraft
     */
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

    /**
     * @brief List all possible paths for a fleet given their arrival times
     * 
     * @param starts    Starting poses
     * @param ends      Ending poses (ground referential)
     * @param stats     Aircraft statistics
     * @param timeslots Several possible times of arrival for each aircraft
     * @param wind_x    Wind, X component
     * @param wind_y    Wind, Y component
     * @return ListOfPossibilities List of possible paths per aircraft
     */
    ListOfPossibilities list_all_possibilities(
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, const std::vector<std::vector<double>>& timeslots, double wind_x, double wind_y) const
    {
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == timeslots.size());
#endif
        uint N = starts.size();

        ListOfPossibilities output(N);

        for(uint i = 0; i < N; i++)
        {
            
            Pose3D base_end = ends[i];
            std::vector<double> possible_times = timeslots[i];
            for(double target_time : possible_times)
            {
                // Modify target according to estimated travel time
                base_end.x -= target_time*wind_x;
                base_end.y -= target_time*wind_y;
                double target_len = target_time*stats[i].airspeed;

                // Add possibilities to aircraft i
                auto possibilities = generate_possible_paths(starts[i],base_end,stats[i],target_len);
                output[i].insert(output[i].end(),
                    std::make_move_iterator(possibilities.begin()),
                    std::make_move_iterator(possibilities.end()));
            }
        }

        return output;
    }


public:
    int verbosity;
    std::ostream& datalog;

    AbstractFleetPlanner(double _prec, double _max_r_dur, int verb, std::ostream& output_stream) : 
        precision_tol(_prec), maximal_relative_duration(_max_r_dur), verbosity(verb), datalog(output_stream) {}

    AbstractFleetPlanner(double _prec, double _max_r_dur, int verb) :
        precision_tol(_prec), maximal_relative_duration(_max_r_dur), verbosity(verb), datalog(std::cout) {}

    AbstractFleetPlanner(double _prec, double _max_r_dur) :
        precision_tol(_prec), maximal_relative_duration(_max_r_dur), verbosity(DubinsFleetPlanner_VERBOSE), datalog(std::cout) {}

    /**
     * @brief  Solve the pathfinding problem for given target times
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @tparam distance_function    Function computing the distance between two path (usually underline the separation one)
     * @param extra     Extra info regarding the solving process
     * @param starts    List of starting poses 
     * @param ends      List of ending poses
     * @param stats     Statistics for aircraft
     * @param min_sep   Minimal required separation
     * @param times     Target time for each aircraft
     * @param wind_x    X component of the wind vector. Default to no wind.
     * @param wind_y    Y component of the wind vector. Default to no wind.
     * @param threads   Number of threads to use for solving. 
     *                      If nonpositive, disable threading
     *                      If 1, set the number of threads to std::thread::hardware_concurrency()
     *                      Otherwise, use the given value
     * @param init_ref_model If true, initialize the `ref_model` attribute (required for using the HiGHS solver)
     * @param obstacle_paths Paths acting as obstacles (representing other aircraft)
     * @param obstacle_stats Flight characteristics of the obstacle aircraft
     * @return std::pair<SharedDubinsResults,std::set<uint>> Returns a pair, with first a solution (if it exists) and second
     *  the set of aircraft IDs with no possible paths
     */
    template<Dubins::DubinsSeparationFunction separation_function, Dubins::DubinsDistanceFunction distance_function>
    std::pair<SharedDubinsResults,std::set<uint>> solve_once(
        ExtraPPResults& extra,
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, double min_sep,
        const std::vector<double>& times,
        double wind_x = 0., double wind_y = 0.,
        int threads = 0, bool init_ref_model=true,
        const std::vector<std::shared_ptr<Dubins>>& obstacle_paths = {}, const std::vector<AircraftStats>& obstacle_stats = {}
    ) const
    {
        // Timing
        chrono::thread_clock clk;
        auto start = clk.now();

        // Init variables
        uint N = starts.size();
        if (threads == 0)
        {
            threads = std::thread::hardware_concurrency();
        }

        extra.threads = std::max(threads,0);
        extra.possible_paths_num = max_path_num();
        min_sep = std::abs(min_sep);
         
        // Check safety
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == times.size());
        assert(obstacle_paths.size() == obstacle_stats.size());

        for(uint i = 0; i < N; i++)
        {
            for(uint j = i+1; j < N; j++)
            {
                assert(pose_dist_XY(starts[i],starts[j]) > min_sep);
            }
        }
#endif

        ListOfPossibilities possibilities = list_all_possibilities(starts,ends,stats,times,wind_x,wind_y);

        filter_colliding_paths<distance_function>(possibilities,stats,obstacle_paths,obstacle_stats,min_sep);

        std::set<uint> nopaths_acs = acs_without_paths(possibilities);

        SharedDubinsResults sol;

        if (nopaths_acs.size() == 0)
        {
            if (threads > 1)
            {
                if (init_ref_model)
                {
                    ref_model = new Highs();
                    setup_base_model(ref_model,starts.size(),max_path_num(),verbosity);
                }

                sol = find_solution_parallel<separation_function,distance_function>(possibilities,stats,min_sep,max_path_num(),threads,
                    verbosity >= DubinsFleetPlanner_VERY_VERBOSE,
                    *times.begin());
            }
            else
            {
                UniqueDubinsResults tmp_sol = find_solution<separation_function>(possibilities,stats,min_sep);

                if (tmp_sol.has_value())
                {
                    sol = make_shared(tmp_sol.value());
                }
                else
                {
                    sol = std::nullopt;
                }
            }
        } 
        else
        {
            sol = std::nullopt;

            if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
            {
                std::cout   << "AC numbers : ";
                for(uint id : nopaths_acs)
                {
                    std::cout << id << ", ";
                } 
                std::cout << " have no path available..." << std::endl;
            }
        }

        auto end = clk.now();

        extra.success = sol.has_value();
        extra.duration = end - start;

        return std::make_pair(sol,nopaths_acs);
    }

    /**
     * @brief Solve the pathfinding problem using Dubins paths with known differences between time of arrivals, over a range of
     * possible times
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @tparam distance_function    Function computing the minimal distance location and value between two trajectories
     * @param extra         Extra information regarding the solving process
     * @param starts        List of starting poses 
     * @param ends          List of ending poses
     * @param stats         Statistics for aircraft
     * @param min_sep       Minimal required separation
     * @param delta_t       Wanted time separation between arrivals.
     * @param wind_x        X component of the wind vector. Default to no wind.
     * @param wind_y        Y component of the wind vector. Default to no wind.
     * @param max_iters     Maximal number of iterations before aborting research. Default to 300
     * @param weave_iters   Number of iterations to add between two failed samples. Default to 2
     * @param min_weave_delta   Minimal value between two iterations for weaving between them. Default to 1.
     * @param threads       Number of threads to use for solving. 
     *                          If nonpositive, disable threading
     *                          If 1, set the number of threads to std::thread::hardware_concurrency()
     *                          Otherwise, use the given value
     * @param obstacle_paths Paths acting as obstacles (representing other aircraft)
     * @param obstacle_stats Flight characteristics of the obstacle aircraft
     * @return SharedDubinsResults
     */
    template<Dubins::DubinsSeparationFunction separation_function, Dubins::DubinsDistanceFunction distance_function>
    SharedDubinsResults solve(
        ExtraPPResults& extra,
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, double min_sep,
        const std::vector<double>& delta_t,
        double wind_x = 0., double wind_y = 0.,
        uint max_iters = 300, uint weave_iters = 2,
        double min_weave_delta =1., double min_weave_frac =0.0001,
        int threads = -1,
        int max_time_s = 60,
        const std::vector<std::shared_ptr<Dubins>>& obstacle_paths = {}, const std::vector<AircraftStats>& obstacle_stats = {}
    ) const
    {
        // Timing
        chrono::milliseconds max_dur_s(static_cast<long int>(max_time_s*1000));
        auto start = chrono::thread_clock::now();
        auto max_date = start + max_dur_s;

        // Variables preprocessing
        uint N = starts.size();
        if (threads == 0)
        {
            threads = std::thread::hardware_concurrency();
        }        
        
        extra.threads = std::max(threads,0);
        extra.possible_paths_num = max_path_num();

        if (verbosity >= DubinsFleetPlanner_VERBOSE)
        {
            std::cout << "Using " << extra.threads << " threads" << std::endl;
        }

        min_sep = std::abs(min_sep);
         
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == delta_t.size() +1);
        assert(obstacle_paths.size() == obstacle_stats.size());

        for(uint i = 0; i < N; i++)
        {
            for(uint j = i+1; j < N; j++)
            {
                assert(pose_dist_XY(starts[i],starts[j]) > min_sep);
            }
        }
#endif


        double min_time = maxmin_dubins_traveltime(starts,ends,stats,delta_t,wind_x,wind_y);

        extra.initial_path_time = min_time;

        double max_time = min_time * maximal_relative_duration;

        min_weave_delta = std::max(min_weave_delta,min_weave_frac * max_time);

        std::list<SampleCase> samples = generate_linspace(min_time,max_time,weave_iters,true,true);
        auto iter = samples.begin();

        uint iter_count = 0;
        SharedDubinsResults best_sol;
        double best_time = max_time;

        auto solve_timepoint = start;
        double solver_rate = INFINITY;

        while(iter_count < max_iters && samples.size() > 1 && chrono::thread_clock::now() < max_date)
        {
            iter_count++;

            // Remove cases already done
            while(iter != samples.end() && iter->done)
            {
                iter++;
            }

            // If nothing left, regenerate
            if (iter == samples.end())
            {
                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << "Adding new samples, going from " << samples.size() << " to ";
                }

                if (samples.size() <= 1)
                {
                    break;
                }
                bool new_samples = weave_samples(samples,weave_iters,min_weave_delta);

                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << samples.size() << std::endl;
                }

                if (!new_samples)
                {
                    if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
                    {
                        std::cout << "No new samples added; terminating early" << std::endl;
                    }
                    break;
                }

                iter = samples.begin();
                continue;
            }

            // Found a valid sample to test!

            double target_time = iter->val;

            if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
            {
                std::cout   << "Starting iteration number " << iter_count 
                            << " (target time: " << target_time << " )" << std::endl;
            }

            // Compute arrival time for each AC and solve
            std::vector<double> times = compute_arrival_times(delta_t,target_time);


            SharedDubinsResults sol;
            std::set<uint> nopaths_acs;
            std::tie(sol,nopaths_acs) = solve_once<separation_function,distance_function>(
                extra,starts,ends,
                stats,
                min_sep,times,
                wind_x,wind_y,
                threads,false,
                obstacle_paths,obstacle_stats);

            iter->NoPathsACs = nopaths_acs;
            iter->done = true;

            if (sol.has_value())
            {
                iter->success = true;

                best_sol = sol.value();

                auto now = chrono::thread_clock::now();
                chrono::nanoseconds dt = now - solve_timepoint;
                double rate = (best_time - target_time)/(static_cast<double>(dt.count())/1e9);

                solver_rate = std::min(solver_rate,rate);

                solve_timepoint = now;
                best_time = target_time;

                if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
                {
                    std::cout << "Found a solution at " << best_time << std::endl;
                }

                // Remove all values after the best solution so far
                auto next = iter;
                next++;
                
                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << "Dropping points from " << samples.size();
                }

                samples.erase(next,samples.end());

                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << " to " << samples.size();
                }

                // Weave
                if (samples.size() <= 1)
                {
                    break;
                }
                bool new_samples = weave_samples(samples,weave_iters,min_weave_delta);

                if (verbosity >= DubinsFleetPlanner_VERYVERY_VERBOSE)
                {
                    std::cout << " then weaved to " << samples.size() << std::endl;
                }

                // Reset iterator
                iter = samples.begin();
            }
            else
            {
                iter->success = false;
            }
            iter++;
        }

        extra.duration = chrono::thread_clock::now() - start;

        if (verbosity >= DubinsFleetPlanner_VERBOSE && extra.duration > max_dur_s)
        {
            std::cout << "WARNING: Timeout stop!" << std::endl;
        }

        extra.success = best_sol.has_value();

        extra.worst_improvement_rate = solver_rate;
        extra.iterations        = iter_count;
        extra.final_path_time   = best_time;
        return best_sol;
    }

    /**
     * @brief Solve the pathfinding problem using Dubins paths with fixed arrival times
     * 
     * @tparam separation_function  Function to assert if two paths are separated or not
     * @tparam distance_function    Function computing the minimal distance location and value between two trajectories
     * @param extra         Extra information regarding the solving process
     * @param starts        List of starting poses 
     * @param ends          List of ending poses
     * @param stats         Statistics for aircraft
     * @param timeslots     For each aircraft, its possible arrival times (should be sorted by preferred arrival time)
     * @param delta_t       Wanted time separation between arrivals.
     * @param wind_x        X component of the wind vector. Default to no wind.
     * @param wind_y        Y component of the wind vector. Default to no wind.
     * @param threads       Number of threads to use for solving. 
     *                          If nonpositive, disable threading
     *                          If 1, set the number of threads to std::thread::hardware_concurrency()
     *                          Otherwise, use the given value
     * @param obstacle_paths Paths acting as obstacles (representing other aircraft)
     * @param obstacle_stats Flight characteristics of the obstacle aircraft
     * @return SharedDubinsResults
     */
    template<Dubins::DubinsSeparationFunction separation_function, Dubins::DubinsDistanceFunction distance_function>
    std::pair<SharedDubinsResults,std::set<uint>> solve_with_timeslots(
        ExtraPPResults& extra,
        const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
        const std::vector<AircraftStats>& stats, double min_sep,
        const std::vector<std::vector<double>>& timeslots,
        double wind_x = 0., double wind_y = 0.,
        int threads = -1,
        const std::vector<std::shared_ptr<Dubins>>& obstacle_paths = {}, const std::vector<AircraftStats>& obstacle_stats = {}
    ) const
    {
        // Timing
        chrono::process_real_cpu_clock clk;
        auto start = clk.now();

        // Init variables
        uint N = starts.size();
        if (threads == 0)
        {
            threads = std::thread::hardware_concurrency();
        }

        extra.threads = std::max(threads,0);
        extra.possible_paths_num = max_path_num();
        min_sep = std::abs(min_sep);

        
        // Check safety
    #if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
        assert(starts.size() == ends.size());
        assert(starts.size() == stats.size());
        assert(starts.size() == timeslots.size());
        assert(obstacle_paths.size() == obstacle_stats.size());
    #endif

        double maxmin_ts = 0.;
        size_t max_timeslots = timeslots[0].size();
        for(auto ts : timeslots)
        {
            maxmin_ts = std::max(maxmin_ts,*std::min_element(ts.begin(),ts.end()));
            max_timeslots = std::max(max_timeslots,ts.size());
        }
        extra.initial_path_time = maxmin_ts;

        ListOfPossibilities possibilities = list_all_possibilities(starts,ends,stats,timeslots,wind_x,wind_y);

        filter_colliding_paths<distance_function>(possibilities,stats,obstacle_paths,obstacle_stats,min_sep);


        std::set<uint> nopaths_acs = acs_without_paths(possibilities);

        SharedDubinsResults sol;

        if (nopaths_acs.size() == 0)
        {
            if (threads > 1)
            {
                ref_model = new Highs();
                setup_base_model(ref_model,starts.size(),max_timeslots*max_path_num(),verbosity);

                sol = find_solution_parallel<separation_function,distance_function>(
                    possibilities,stats,min_sep,max_timeslots*max_path_num(),threads, verbosity >= DubinsFleetPlanner_VERY_VERBOSE);
            }
            else
            {
                UniqueDubinsResults tmp_sol = find_solution<separation_function>(possibilities,stats,min_sep);

                if (tmp_sol.has_value())
                {
                    sol = make_shared(tmp_sol.value());
                }
                else
                {
                    sol = std::nullopt;
                }
            }
        } 
        else
        {
            sol = std::nullopt;

            if (verbosity >= DubinsFleetPlanner_VERY_VERBOSE)
            {
                std::cout   << "AC numbers : ";
                for(uint id : nopaths_acs)
                {
                    std::cout << id << ", ";
                } 
                std::cout << " have no path available..." << std::endl;
            }
        }

        auto end = clk.now();

        extra.success = sol.has_value();
        extra.duration = end - start;

        if (sol.has_value())
        {
            uint i = 0;
            double max_duration = 0.;
            for(const auto& d : sol.value())
            {
                const auto& s = stats[i];
                max_duration = std::max(max_duration,d->get_duration(s.airspeed));
                i++;
            }
            extra.final_path_time = max_duration;
        }


        return std::make_pair(sol,nopaths_acs);
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

    BasicDubinsFleetPlanner(double _prec, double _max_r_dur, int verb, std::ostream& output_stream) :
        AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream) {}
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

    ExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb, std::ostream& output_stream)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream), start_lengths(_slens), end_lengths(_elens) {}

};

/**
 * @brief Plannification based on the 8 basic Dubins paths extended by basic primitives
 * 
 */
class BaseExtendedDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<double> start_lengths,end_lengths; // Finite list of possible lengths for starting and ending extra path

    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        return generate_all_fitted_base_extended(start,end,start_lengths,end_lengths,stats.climb,stats.turn_radius,target_len,precision_tol);
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
    BaseExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens)
        : AbstractFleetPlanner(_prec,_max_r_dur), start_lengths(_slens), end_lengths(_elens) {}

    BaseExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb), start_lengths(_slens), end_lengths(_elens) {}

    BaseExtendedDubinsFleetPlanner(double _prec, double _max_r_dur,
        const std::vector<double>& _slens, const std::vector<double>& _elens, int verb, std::ostream& output_stream)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream), start_lengths(_slens), end_lengths(_elens) {}

};

/**
 * @brief Plannification based on the 6 fundamental Dubins paths using the minimal turn radius,
 * extended by well-chosen straight at the start and end
 * 
 */
class LineExtendedDubinsFleetPlanner : public AbstractFleetPlanner
{
private:
    std::vector<double> ratios;

    std::vector<std::unique_ptr<Dubins>> generate_possible_paths(const Pose3D& start, const Pose3D& end, const AircraftStats& stats, double target_len) const
    {
        std::vector<std::unique_ptr<Dubins>> output = generate_line_extended_base(start,end,stats.climb,stats.turn_radius,target_len,precision_tol);
        std::vector<std::unique_ptr<Dubins>> curvature_extended = fit_possible_baseDubins(stats.climb,stats.turn_radius,start,end,target_len,precision_tol);
        
        for(auto &c : curvature_extended)
        {
            output.push_back(std::move(c));
        }

        return output;
    }

    uint max_path_num() const
    {
        return 3*6+8; // Three types of extensions: start,end,boths for the 6 original Dubins paths, as well as the 8 curvature adjusted Dubins
    } 

public:
    LineExtendedDubinsFleetPlanner(double _prec, double _max_r_dur, const std::vector<double>& rs)
        : AbstractFleetPlanner(_prec,_max_r_dur),ratios(rs){}

    LineExtendedDubinsFleetPlanner(double _prec, double _max_r_dur, const std::vector<double>& rs,
        int verb)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb),ratios(rs){}

    LineExtendedDubinsFleetPlanner(double _prec, double _max_r_dur, const std::vector<double>& rs,
        int verb, std::ostream& output_stream)
        : AbstractFleetPlanner(_prec,_max_r_dur,verb, output_stream),ratios(rs){}

};