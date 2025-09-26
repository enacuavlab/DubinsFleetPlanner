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

#include "ConflictList.hpp"

// ---------------------------------------- Util functions ---------------------------------------- //

template<typename F>
static inline bool generic_check_compatibility(F are_separated_function, const Dubins& d1, const Dubins& d2, const AircraftStats& s1, const AircraftStats& s2, double sep)
{
    double duration = std::min(d1.get_duration(s1.airspeed),d2.get_duration(s2.airspeed));
    return d1.is_valid() && d2.is_valid() && are_separated_function(d1,d2,s1.airspeed,s2.airspeed,duration,sep,DubinsFleetPlanner_PRECISION);
}

// ---------------------------------------- Single thread functions ---------------------------------------- //

template<typename F>
std::vector<Conflict_T> generic_compute_separations(F are_separated_fun, const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep)
{
    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    std::vector<Conflict_T> output;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            const std::vector<std::unique_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::unique_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d1_paths[p2];

                    if (!generic_check_compatibility(are_separated_fun,d1,d2,d1_stats,d2_stats,sep))
                    {
                        output.push_back({
                            i,j,p1,p2
                        });
                    }
                }
            }

        }
    }

    return output;
}

// ---------------------------------------- Multi thread functions ---------------------------------------- //

template<typename F>
std::vector<Conflict_T> _subtask_compute_separations(F are_separated_fun,
    const SharedListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats,
    double sep, uint start_i, uint start_j, uint ac_num)
{

    uint N = list_of_possibilites.size();
    uint done = 0;

    uint i = start_i;
    uint j = start_j;

    // std::cout << "Thread " << std::this_thread::get_id() << " : ( " << start_i << " , " << start_j << " , " << ac_num << " )" << std::endl;

    std::vector<Conflict_T> output;

    while(i < N)
    {
        while(j < N)
        {
            const std::vector<std::shared_ptr<Dubins>>& d1_paths = list_of_possibilites[i];
            const std::vector<std::shared_ptr<Dubins>>& d2_paths = list_of_possibilites[j];

            const AircraftStats& d1_stats = stats[i];
            const AircraftStats& d2_stats = stats[j];

            uint P1 = d1_paths.size();
            uint P2 = d2_paths.size();

            for(uint p1 = 0; p1 < P1; p1++)
            {
                for(uint p2 = 0; p2 < P2; p2++)
                {
                    const Dubins& d1 = *d1_paths[p1];
                    const Dubins& d2 = *d2_paths[p2];

                    // std::cout   << "Thread " << std::this_thread::get_id() << " : "
                    //             << "( " << i << " , " << p1 << " , " << j << " , " << p2 << " ) [ "
                    //             << d1.get_type_abbr() << " VS " << d2.get_type_abbr() << " ]" << std::endl; 
                            

                    if (!generic_check_compatibility(are_separated_fun,d1,d2,d1_stats,d2_stats,sep))
                    {
                        output.push_back({
                            i,j,p1,p2
                        });
                    }
                }
            }

            done++;
            if (done >= ac_num)
            {
                return output;
            }

            j++;
        }

        i++;
        j = i+1;
    }

    return output;
}


template<typename F>
std::vector<Conflict_T> generic_parallel_compute_separations(uint THREADS, F are_separated_fun, const SharedListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep)
{
    assert(THREADS > 0);

    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif

    uint tasks_to_do = N*(N-1)/2;
    std::vector<Conflict_T> output;

    uint tasks_per = tasks_to_do/THREADS;
    uint thread_started = 0;
    uint tasks_given = 0;

    uint start_i,start_j;

    std::vector<std::future<std::vector<Conflict_T>>> futures;

    for(uint i = 0; i < N; i++)
    {
        for(uint j = i+1; j < N; j++)
        {
            if (tasks_given >= tasks_per)
            {
                futures.push_back(
                    std::async(
                        std::launch::async,
                        _subtask_compute_separations<F>,
                        are_separated_fun,list_of_possibilites,stats,sep,
                        start_i,start_j,tasks_given
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
        output = _subtask_compute_separations(are_separated_fun,
            list_of_possibilites,stats,
            sep,start_i,start_j,tasks_given);
    }

    for(auto& res : futures)
    {
        res.wait();
        auto res_val = res.get();
        output.insert(output.end(),res_val.begin(), res_val.end());
    }

    return output;
}


// ---------------------------------------- Specialized implementations ---------------------------------------- //

std::vector<Conflict_T> compute_XY_separations(const ListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep)
{
    return generic_compute_separations(Dubins::are_XY_separated<true>,list_of_possibilites,stats,sep);
}

std::vector<Conflict_T> parallel_compute_XY_separations(const SharedListOfPossibilities& list_of_possibilites, const std::vector<AircraftStats>& stats, double sep, uint THREADS)
{
    return generic_parallel_compute_separations(
        THREADS,
        Dubins::are_XY_separated<true>,
        list_of_possibilites,stats,sep
    );
}

// ---------------------------------------- LP Solver ---------------------------------------- //

void setup_base_model(Highs& highs, uint AC_count, uint max_paths_count)
{
    // -- General Options
    highs.setOptionValue("output_flag",true);
    highs.setOptionValue("presolve","on");
    highs.setOptionValue("parallel","on");
    highs.setOptionValue("log_file","highs.log");
    highs.setOptionValue("mip_abs_gap", 1-1e-5); // Since all variables in the objective are binary, it should stop when the absolute gap is below 1
    // highs.setOptionValue('random_seed', SEED)

    // Define variables with bounds (01-LP problem)
    uint var_num = AC_count * max_paths_count; 

    std::vector<double> lowers(var_num);
    std::fill(lowers.begin(),lowers.end(),0.);
    std::vector<double> uppers(var_num);
    std::fill(uppers.begin(),uppers.end(),1.);


    // Objective : chose lower path index for each AC (assume paths ordered by time)
    std::vector<double> costs(var_num);
    for(uint ac = 0; ac < AC_count; ac++)
    {
        for(uint p_index = 0; p_index < max_paths_count; p_index++)
        {
            costs[ac*max_paths_count+p_index] = p_index+1.;
        }
    }

    highs.addCols(var_num,costs.data(),lowers.data(),uppers.data(),0,nullptr,nullptr,nullptr);


    std::vector<HighsVarType> integrality(var_num);
    std::fill(integrality.begin(),integrality.end(),HighsVarType::kInteger);
    highs.changeColsIntegrality(0,var_num-1,integrality.data());


    // -- Constraints 

    std::vector<int> indices(max_paths_count);


    // Unique path for each AC
    for(uint ac_id = 0; ac_id < AC_count; ac_id++)
    {
        for(uint i = 0; i < max_paths_count; i++)
        {
            indices[i] = ac_id*max_paths_count+i;
        }

        highs.addRow(1.,1.,max_paths_count,indices.data(),uppers.data());
    }
}

std::optional<std::vector<std::shared_ptr<Dubins>>> find_pathplanning_LP_solution(
    SharedListOfPossibilities& list_of_possibilites,
    const std::vector<AircraftStats>& stats,
    const std::vector<Conflict_T>& conflicts,
    uint max_path_num,
    Highs* preset_model)
{

    uint N = list_of_possibilites.size();

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(list_of_possibilites.size() == stats.size());
#endif
    
    // -- Use preset model

    Highs model;

    if (preset_model == nullptr)
    {
        setup_base_model(model,N,max_path_num);
    }
    else
    {
        assert(HighsStatus::kOk == model.passOptions(preset_model->getOptions()));
        assert(HighsStatus::kOk == model.passModel(preset_model->getModel()));
    }

    // -- Remove impossible paths by adding constraints

    for(uint ac_id = 0; ac_id < list_of_possibilites.size(); ac_id++)
    {
        auto& path = list_of_possibilites[ac_id];
        if (path.size() < max_path_num)
        {
            std::vector<int> indices;
            std::vector<double> values;
            uint base_id = ac_id*max_path_num;
            for(uint p_i = path.size(); p_i < max_path_num; p_i++)
            {
                indices.push_back(base_id+p_i);
                values.push_back(1.);
            }
            model.addRow(-1,0.,max_path_num-path.size(),indices.data(),values.data());
        }
    }

    // -- Add conflicts

    for(const Conflict_T& c : conflicts)
    {
        uint indices[] = {
            c.ac_id1*max_path_num + c.path_id1,
            c.ac_id2*max_path_num + c.path_id2
        };

        double values[] = {
            1.,1.
        };

        model.addRow(0,1.,2,(const int*)indices,values);
    }

    // -- Solve

    assert(model.run() == HighsStatus::kOk);


    if (model.getModelStatus() == HighsModelStatus::kOptimal)
    {
        std::vector<std::shared_ptr<Dubins>> output;
        uint i = 0;
        for(double val : model.getSolution().col_value)
        {
            if (val > 0.5)
            {
                uint ac_id = i / max_path_num;
                uint path_id = i % max_path_num;
                output.push_back(list_of_possibilites[ac_id][path_id]);
            }

            i++;
        }
        return output;
    }
    else
    {
        return std::nullopt;
    }
}
