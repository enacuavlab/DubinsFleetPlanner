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

#include "ConflictMatrix.hpp"

// ---------------------------------------- Util functions ---------------------------------------- //

uint ac_index_map(uint i, uint j, uint N)
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(i < j && j < N);
#endif

    return N*i+j-((i+1)*(1*2))/2;
}

uint path_index_map(uint i, uint j, uint P1, uint P2)
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(i < P1 && j < P2);
#endif
    return i*P2 + j;
}

template<typename F>
static inline bool generic_check_compatibility(F are_separated_function, const Dubins& d1, const Dubins& d2, const AircraftStats& s1, const AircraftStats& s2, double sep)
{
    double duration = std::min(d1.get_duration(s1.airspeed),d2.get_duration(s2.airspeed));
    return d1.is_valid() && d2.is_valid() && are_separated_function(d1,d2,s1.airspeed,s2.airspeed,duration,sep);
}

// ---------------------------------------- Single thread functions ---------------------------------------- //

template<typename F>
aapp_feasability_matrix_T generic_compute_separations(F are_separated_fun, const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep)
{
    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    aapp_feasability_matrix_T output(N*(N-1)/2);
    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            uint index = ac_index_map(i,j,N);

            const std::vector<std::unique_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::unique_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            std::vector<bool> paths_matrix(P1 * P2);

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d1_paths[p2];


                    uint p_index = path_index_map(p1,p2,P1,P2);

                    paths_matrix[p_index] = generic_check_compatibility(are_separated_fun,d1,d2,d1_stats,d2_stats,sep);
                }
            }

        }
    }

    return output;
}

// ---------------------------------------- Multi thread functions ---------------------------------------- //

template<typename F>
bool _subtask_compute_separations(F are_separated_fun,
    const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep,
    aapp_feasability_matrix_T& results,
    uint start_i, uint start_j, uint ac_num)
{
    uint N = list_of_possibilites.size();
    uint done = 0;

    uint i = start_i;
    uint j = start_j;

    while(i < N)
    {
        while(j < N)
        {
            uint index = ac_index_map(i,j,N);

            const std::vector<std::unique_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::unique_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            std::vector<bool> paths_matrix(P1 * P2);


            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d1_paths[p2];


                    uint p_index = path_index_map(p1,p2,P1,P2);

                    paths_matrix[p_index] = generic_check_compatibility(are_separated_fun,d1,d2,d1_stats,d2_stats,sep);
                }
            }

            done++;
            if (done >= ac_num)
            {
                return true;
            }
        }

        i++;
        j = i+1;
    }

    return true;
}


template<typename F>
aapp_feasability_matrix_T generic_parallel_compute_separations(uint THREADS, F are_separated_fun, const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep)
{
    assert(THREADS > 0);

    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    uint tasks_to_do = N*(N-1)/2;
    aapp_feasability_matrix_T output(tasks_to_do);

    uint tasks_per = tasks_to_do/THREADS;
    uint thread_started = 0;
    uint tasks_given = 0;

    uint start_i,start_j;

    std::vector futures;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            if (tasks_given >= tasks_per)
            {
                futures.push_back(
                    std::async(
                        std::launch::async,
                        _subtask_compute_separations,
                        are_separated_fun,list_of_possibilites,stats,sep,
                        output,start_i,start_j,tasks_given
                    )
                );
                tasks_to_do -= tasks_given;
                thread_started++;
                tasks_given = 0;
            }

            if (tasks_given == 0)
            {
                start_i = i;
                start_j = j;
            }

            tasks_given++;

        }
    }

    if (tasks_given > 0)
    {
        _subtask_compute_separations(are_separated_fun,list_of_possibilites,stats,sep,
                        output,start_i,start_j,tasks_given);
    }

    for(auto& res : futures)
    {
        res.wait();
    }
    return output;
}


// ---------------------------------------- Specialized implementations ---------------------------------------- //

aapp_feasability_matrix_T compute_XY_separations(const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep)
{
    return generic_compute_separations(Dubins::are_XY_separated<true>,list_of_possibilites,stats,sep);
}

aapp_feasability_matrix_T parallel_compute_XY_separations(const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep, uint THREADS)
{
    return generic_parallel_compute_separations(
        THREADS,
        Dubins::are_XY_separated<true>,
        list_of_possibilites,stats,sep
    );
}

// ---------------------------------------- LP Solver ---------------------------------------- //

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
 *   (Note: )
 * 
 * @param AC_count Number of aircraft
 * @param max_paths_count Maximum number of possible paths per aircraft
 * @return Highs A preconfigured model
 */
Highs generate_base_model(uint AC_count, uint max_paths_count)
{

    Highs highs;

    // Genral Options
    highs.setOptionValue('output_flag',true);
    highs.setOptionValue('mip_abs_gap', 1-1e-5) // Since all variables in the objective are binary, it should stop when the absolute gap is below 1
    // highs.setOptionValue('random_seed', SEED)

    // Define variables with bounds (01-LP problem)
    uint var_num = AC_count * max_paths_count; 

    std::vector<double> lowers(var_num);
    std::fill(lowers.begin(),lowers.end(),0.);
    std::vector<double> uppers(var_num);
    std::fill(uppers.begin(),uppers.end(),1.);

    highs.addVars(var_num,lowers.data(),uppers.data());

    std::vector<HighsVarType> integrality(var_num);
    std::fill(integrality.begin(),integrality.end(),HighsVarType::kInteger);
    highs.changeColsIntegrality(0,var_num-1,integrality.data());

    // -- Constraints 

    
}

std::vector<std::unique_ptr<Dubins>> find_pathplanning_LP_soltuion(
    const ListOfPossibilities& list_of_possibilites,
    const std::vector<AircraftStats>& stats,
    const aapp_feasability_matrix_T& feasability)
{

    
}
