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

// ---------- Local template for computing separation between shapes ---------- //

/**
 * @brief Check is two basic path elements are separated based on the XY euclidean distance
 * 
 * @tparam m1 Type of the first element
 * @tparam m2 Type of the second element
 * @tparam geometric_filtering Toggle geometric pre-filtering
 * @param p1 First path
 * @param p2 Second path
 * @param duration Time duration of the section to study
 * @param min_sep Minial required distance
 * @param tol Precision with respect
 * @return true Path elements are separated
 * @return false Path are not separated
 */
template<DubinsMove m1, DubinsMove m2, bool geometric_filtering, bool use_derivatives=true>
static bool check_XY_separation(const PathShape<m1>& p1, const PathShape<m2>& p2, double duration, double min_sep, double tol)
{
    if (duration < tol)
    {
        Pose3D p1_start = initial_pose(p1);
        Pose3D p2_start = initial_pose(p2);
        return pose_dist_XY(p1_start,p2_start) > min_sep;
    }

    if ((m1 == STRAIGHT) && (m2 == STRAIGHT))
    {
        auto loc_val_dist = temporal_XY_dist<m1,m2,use_derivatives>(p1,p2,duration,tol);
        if (loc_val_dist.second < min_sep)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        if (geometric_filtering)
        {
            double geo_dist = geometric_XY_dist(p1,p2,duration);
            if (geo_dist < min_sep)
            {
                auto loc_val_dist = temporal_XY_dist<m1,m2,use_derivatives>(p1,p2,duration,tol);
                if (loc_val_dist.second < min_sep)
                {
                    return false;
                }
            }
        }
        else
        {
            auto loc_val_dist = temporal_XY_dist<m1,m2,use_derivatives>(p1,p2,duration,tol);
            if (loc_val_dist.second < min_sep)
            {
                return false;
            }
        }
        return true;
    }
}

/**
 * @brief Compute the minimal XY distance between two paths elements 
 * 
 * @tparam m1 Type of the first element
 * @tparam m2 Type of the second element
 * @param p1 First path
 * @param p2 Second path
 * @param duration Time duration of the section to study
 * @param min_sep Minial required distance
 * @param tol Precision with respect
 * @return A pair: Location and value of the minimal distance between two paths
 */
template<DubinsMove m1, DubinsMove m2, bool geometric_filtering, bool use_derivatives=true>
std::pair<double,double> compute_XY_separation(const PathShape<m1>& p1, const PathShape<m2>& p2, double duration, double min_sep, double tol)
{
    std::pair<double,double> output;

    if (duration < tol)
    {
        Pose3D p1_start = initial_pose(p1);
        Pose3D p2_start = initial_pose(p2);

        output.first = duration/2.;
        output.second = pose_dist_XY(p1_start,p2_start);
    }

    if ((m1 == STRAIGHT) && (m2 == STRAIGHT))
    {
        output = temporal_XY_dist<m1,m2,use_derivatives>(p1,p2,duration,tol);
    }
    else
    {
        if (geometric_filtering)
        {
            double geo_dist = geometric_XY_dist(p1,p2,duration);
            if (geo_dist < min_sep)
            {
                output = temporal_XY_dist<m1,m2,use_derivatives>(p1,p2,duration,tol);
            }
            else
            {
                output.first = duration/2.;
                output.second = geo_dist;
            }
        }
        else
        {
            output = temporal_XY_dist<m1,m2,use_derivatives>(p1,p2,duration,tol);
        }
    }

    return output;
}

// ---------- Separation between Dubins ---------- //

/**
 * @brief For two Dubins paths, compute the sections identifying the part when we stick to a specific basic path
 * 
 * @param p1 One Dubins path
 * @param p2 Other Dubins path
 * @param p1_speed Speed along first path
 * @param p2_speed Speed along second path
 * @param duration Duration for which we follow the path
 * @return std::vector<double> List of junctions timestamps
 */
static std::vector<double> compute_timepoints(const Dubins &p1, const Dubins &p2, double p1_speed, double p2_speed, double &duration)
{
    std::vector<double> timepoints = {0.};
    
    std::vector<double> this_junctions = p1.get_junction_locs();
    std::vector<double> other_junctions = p2.get_junction_locs();
    size_t this_index = 0;
    size_t other_index = 0;

    // Merge the junctions points lists by timestamp until total duration is met
    while ((this_index < this_junctions.size()) && (other_index < other_junctions.size()))
    {
        double this_candidate = this_junctions[this_index] / p1_speed;
        double other_candidate = other_junctions[other_index] / p2_speed;
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

    while ((this_index < this_junctions.size()) && (this_junctions[this_index] / p1_speed < duration))
    {
        timepoints.push_back(this_junctions[this_index] / p1_speed);
        this_index++;
    }

    while ((other_index < other_junctions.size()) && (other_junctions[other_index] / p2_speed < duration))
    {
        timepoints.push_back(other_junctions[other_index] / p2_speed);
        other_index++;
    }

    timepoints.push_back(duration);
    return timepoints;
}

template<DubinsMove m1, DubinsMove m2, bool geometric_separation>
static std::pair<double,double> compute_XY_separation_between_trajectories(
    const Pose3D& start_1, const Pose3D& end_1, double v1, double r1, double h1,
    const Pose3D& start_2, const Pose3D& end_2, double v2, double r2, double h2,
    double duration, double min_dist, double tol
)
{
    return compute_XY_separation<m1,m2,geometric_separation>(
                    compute_params<m1>(start_1,end_1,v1,r1,h1),
                    compute_params<m2>(start_2,end_2,v2,r2,h2),
                    duration,min_dist,tol);
}

static std::pair<double,double> _compute_XY_separation_between_Dubins_on_interval(
    const Dubins& curr, double curr_speed,
    const Dubins& other, double other_speed,
    double min_dist, double tol,
    double t_start, double t_end
)
{
    double curr_turn_radius = curr.get_turn_radius();
    double curr_vspeed      = curr.get_climb();

    double other_turn_radius = other.get_turn_radius();
    double other_vspeed      = other.get_climb();

    Pose3D curr_start   = curr.get_position(t_start , curr_speed);
    Pose3D curr_end     = curr.get_position(t_end   , curr_speed);
    Pose3D other_start  = other.get_position(t_start, other_speed);
    Pose3D other_end    = other.get_position(t_end  , other_speed);
    DubinsMove curr_type    = curr.get_section_type((t_start+t_end)*curr_speed/2);
    DubinsMove other_type   = other.get_section_type((t_start+t_end)*other_speed/2);
    double duration = t_end - t_start;

    std::pair<double,double> output;

    // Double switch-case to go through all pairs of (DubinsMove,DubinsMove)
    switch (curr_type)
    {
    case STRAIGHT:
        switch (other_type)
        {
        case STRAIGHT:
            output = compute_XY_separation_between_trajectories<STRAIGHT,STRAIGHT,true>(
                    curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                    other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                    duration,min_dist,tol);
            break;
            
        case LEFT:
            output = compute_XY_separation_between_trajectories<STRAIGHT,LEFT,true>(
                    curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                    other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                    duration,min_dist,tol);
            break;

        case RIGHT:
            output = compute_XY_separation_between_trajectories<STRAIGHT,RIGHT,true>(
                    curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                    other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                    duration,min_dist,tol);
            break;

        default:
            exit(EXIT_FAILURE);
        }
        break;
        
    case LEFT:
        switch (other_type)
        {
        case STRAIGHT:
            output = compute_XY_separation_between_trajectories<LEFT,STRAIGHT,true>(
                curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                duration,min_dist,tol);
            break;
            
        case LEFT:
            output = compute_XY_separation_between_trajectories<LEFT,LEFT,true>(
                curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                duration,min_dist,tol);
            break;

        case RIGHT:
            output = compute_XY_separation_between_trajectories<LEFT,RIGHT,true>(
                curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                duration,min_dist,tol);
            break;

        default:
            exit(EXIT_FAILURE);
        }
        break;

    case RIGHT:
        switch (other_type)
        {
        case STRAIGHT:
            output = compute_XY_separation_between_trajectories<RIGHT,STRAIGHT,true>(
                curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                duration,min_dist,tol);
            break;
            
        case LEFT:
            output = compute_XY_separation_between_trajectories<RIGHT,LEFT,true>(
                curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                duration,min_dist,tol);
            break;

        case RIGHT:
            output = compute_XY_separation_between_trajectories<RIGHT,RIGHT,true>(
                curr_start,curr_end,curr_speed,curr_turn_radius,curr_vspeed,
                other_start,other_end,other_speed,other_turn_radius,other_vspeed,
                duration,min_dist,tol);
            break;

        default:
            exit(EXIT_FAILURE);
        }
        break;

    default:
        exit(EXIT_FAILURE);
    }

    output.first += t_start;
    return output;
}

std::pair<double,double> Dubins::XY_distance_to(const Dubins& other, double this_speed, double other_speed, 
        double duration, double min_dist, double tol, std::optional<double> hotstart) const
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert((this_speed > 0) && (other_speed > 0));
    assert((this->is_valid()) && (other.is_valid()));
#endif

    std::vector<double> timepoints = compute_timepoints(*this, other, this_speed, other_speed, duration);

    double this_turn_radius = this->get_turn_radius();
    double this_vspeed      = this->get_climb();

    double other_turn_radius = other.get_turn_radius();
    double other_vspeed      = other.get_climb();

    std::pair<double,double> output = {0.,INFINITY};


    size_t index; // Path section index for hot start

    if (hotstart.has_value())
    {
        index = binary_search(timepoints,hotstart.value());
        std::pair<double,double> current = _compute_XY_separation_between_Dubins_on_interval(
            *this,this_speed,other,other_speed,min_dist,tol,timepoints[index],timepoints[index+1]
        );

        if (current.second < min_dist)
        {
            return current;
        }
        else
        {
            output = current;
        }

    }
    else
    {
        index = timepoints.size(); // Return past-the-post value to avoid skipping
    }


    // For each base case, compute the distance
    for(size_t i = 0; i < timepoints.size()-1; i++)
    {
        // Skip the hot start
        if (i == index)
        {
            continue;
        }

        std::pair<double,double> current = _compute_XY_separation_between_Dubins_on_interval(
            *this,this_speed,other,other_speed,min_dist,tol,timepoints[i],timepoints[i+1]
        );

        if (current.second < min_dist)
        {
            return current;
        }
        else if (current.second < output.second)
        {
            output = current;
        }
        
    }

    return output;
}

template<bool geometric_filtering>
bool Dubins::is_XY_separated_from(const Dubins& other, double this_speed, double other_speed, 
        double duration, double min_dist, double tol) const
{
#if defined(DubinsFleetPlanner_ASSERTIONS) && DubinsFleetPlanner_ASSERTIONS > 0
    assert((this_speed > 0) && (other_speed > 0));
    assert((this->is_valid()) && (other.is_valid()));
#endif

    std::vector<double> timepoints = compute_timepoints(*this, other, this_speed, other_speed, duration);

    double this_turn_radius = this->get_turn_radius();
    double this_vspeed      = this->get_climb();

    double other_turn_radius = other.get_turn_radius();
    double other_vspeed      = other.get_climb();

    // For each base case, compute the distance
    for(size_t i = 0; i < timepoints.size()-1; i++)
    {
        std::pair<double,double> current = _compute_XY_separation_between_Dubins_on_interval(
            *this,this_speed,other,other_speed,min_dist,tol,timepoints[i],timepoints[i+1]
        );

        if (current.second < min_dist)
        {
            return false;
        }
    }

    return true;
}

template bool Dubins::is_XY_separated_from<true>(const Dubins& other, double this_speed, double other_speed, 
        double duration, double min_dist, double tol) const;

template bool Dubins::is_XY_separated_from<false>(const Dubins& other, double this_speed, double other_speed, 
        double duration, double min_dist, double tol) const;