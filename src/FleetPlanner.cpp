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

#include "FleetPlanner.hpp"

double maxmin_dubins_traveltime(
    const std::vector<Pose3D>& starts, const std::vector<Pose3D>& ends,
    const std::vector<AircraftStats>& stats, const std::vector<double>& delta_t,
    double wind_x, double wind_y
)
{

#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert(starts.size() == ends.size());
    assert(starts.size() == stats.size());
    assert(starts.size() == delta_t.size()+1);
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
            s,e,
            wind_x,wind_y
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


std::vector<double> compute_arrival_times(const std::vector<double>& dts, double time_ref)
{
    uint N = dts.size()+1;
    std::vector<double> output(N);

    output[0] = time_ref;
    for(uint i = 1; i < N; i++)
    {
        output[i] = output[i-1] + dts[i-1];
    }

    return output;
}

std::tuple<uint,uint> count_min_number_of_valid_paths(const ListOfPossibilities& all_paths)
{
    uint output = all_paths[0].size();
    uint output_loc = 0;

    uint i = 0;

    for(const std::vector<std::unique_ptr<Dubins>>& v : all_paths)
    {
        uint count = 0;
        for(auto& d : v)
        {
            if (d->is_valid())
            {
                count++;
            }
        }

        if (count < output)
        {
            output = count;
            output_loc = i;
        }

        i++;
    }

    return {output,output_loc};
}

std::set<uint> acs_without_paths(const ListOfPossibilities& all_paths)
{
    std::set<uint> output;
    uint ac_id = 0;
    for(const std::vector<std::unique_ptr<Dubins>>& v : all_paths)
    {
        uint count = 0;
        for(auto& d : v)
        {
            if (d->is_valid())
            {
                count++;
            }
        }

        if (count == 0)
        {
            output.insert(ac_id);
        }

        ac_id++;
    }
    
    return output;
}