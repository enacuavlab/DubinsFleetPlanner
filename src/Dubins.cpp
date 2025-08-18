// Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
// 
// This file is part of PH_Spline.
// 
// PH_Spline is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// PH_Spline is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with PH_Spline.  If not, see <https://www.gnu.org/licenses/>.

#include "Dubins.hpp"

#include "ConflictDetection.hpp"

template<bool geometric_filtering=true>
bool Dubins::is_XY_separated_from(const Dubins& other, double this_speed, double other_speed, 
        double duration, double min_dist)
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert((this_speed > 0) && (other_speed > 0));
#endif

    std::vector<double> this_junctions  = this->get_junction_locs();
    std::vector<double> other_junctions = other.get_junction_locs();

    std::vector<double> timepoints = {0.};
    size_t this_index = 0;
    size_t other_index = 0;

    // Merge the junctions points lists by timestamp until total duration is met
    while((this_index < this_junctions.size()) && (other_index < other_junctions.size()))
    {
        double this_candidate   = this_junctions[this_index]/this_speed;
        double other_candidate  = other_junctions[other_index]/other_speed;
        if ((this_candidate > duration) && (other_candidate > duration))
        {
            break;
        }

        if (this_candidate < other_candidate)
        {
            timepoints.push_back(this_candidate);
            this_index++;
        }
        else
        {
            timepoints.push_back(other_candidate);
            other_index++;
        }
    }

    while((this_index < this_junctions.size()) && (this_junctions[this_index]/this_speed < duration))
    {
        timepoints.push_back(this_junctions[this_index]/this_speed);
        this_index++;
    }

    while((other_index < other_junctions.size()) && (other_junctions[other_index]/other_speed < duration))
    {
        timepoints.push_back(other_junctions[other_index]/other_speed);
        other_index++;
    }

    timepoints.push_back(duration);

    double this_turn_radius = this->get_turn_radius();
    double this_vspeed      = this->get_climb();

    double other_turn_radius = other.get_turn_radius();
    double other_vspeed      = other.get_climb();

    // For each base case, compute the distance
    for(size_t i = 0; i < timepoints.size()-1; i++)
    {
        Pose3D this_start   = this->get_position(timepoints[i],this_speed);
        Pose3D this_end     = this->get_position(timepoints[i+1],this_speed);

        Pose3D other_start  = other.get_position(timepoints[i],other_speed);
        Pose3D other_end    = other.get_position(timepoints[i+1],other_speed);

        DubinsMove this_type    = this->get_section_type((timepoints[i]+timepoints[i+1])*this_speed/2);
        DubinsMove other_type   = other.get_section_type((timepoints[i]+timepoints[i+1])*other_speed/2);

        double section_duration = timepoints[i+1]-timepoints[i];

        if ((this_type == STRAIGHT) && (other_type == STRAIGHT))
        {
            PathShape<STRAIGHT> p1 = compute_params(this_start,this_end,this_speed,this_turn_radius,this_vspeed);
            PathShape<STRAIGHT> p2 = compute_params(other_start,other_end,other_speed,other_turn_radius,other_vspeed);

            auto loc_val_dist = temporal_XY_dist(p1,p2,section_duration);
            if (loc_val_dist.second < min_dist)
            {
                return false;
            }
        }
        else
        {
            PathShape<this_type> p1 = compute_params(this_start,this_end,this_speed,this_turn_radius,this_vspeed);
            PathShape<other_type> p2 = compute_params(other_start,other_end,other_speed,other_turn_radius,other_vspeed);

            double geo_dist = geometric_XY_dist(p1,p2,section_duration);
            if (geo_dist < min_dist)
            {
                auto loc_val_dist = temporal_XY_dist(p1,p2,section_duration);
                if (loc_val_dist.second < min_dist)
                {
                    return false;
                }
            }
        }
        
    }

    return true;
}