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

#include <tuple>

#include "Dubins.hpp"
#include "Primitives.hpp"
#include "utils.hpp"

#if DubinsFleetPlanner_ASSERTIONS > 0
#include <iostream>
#endif

template<DubinsMove fst, DubinsMove snd, DubinsMove trd>
class BaseDubins : public Dubins
{
private:
    static double compute_fst_length(double alpha, double beta, double d);
    static double compute_snd_length(double alpha, double beta, double d);
    static double compute_trd_length(double alpha, double beta, double d);
    static double compute_normalized_length(double alpha, double beta, double d);
    double adjust_length(double target_len, double tol);


    double alpha;   // Start orientation in normalized problem
    double beta;    // End orientation in normalized problem
    double d;       // Distance between endpoints in normalized problem
    Pose3D normalize_transform;     // Encode the transform from standard to normalized problem (add a shift then rotate) 
    double fst_length,snd_length,trd_length;        // Lengths of the three sections of the Dubins path
    Pose3D intermediate_pose_1,intermediate_pose_2; // Poses at the junctions between sections

    
    double _compute_length()
    {
        fst_length = compute_fst_length(alpha,beta,d/turn_radius)*turn_radius;
        snd_length = compute_snd_length(alpha,beta,d/turn_radius)*turn_radius;
        trd_length = compute_trd_length(alpha,beta,d/turn_radius)*turn_radius;
        length = fst_length+snd_length+trd_length;

        return length;
    }

    void normalize()
    {
        auto normalized_values = normalize_poses(start,end);
        alpha   = std::get<0>(normalized_values); 
        beta    = std::get<1>(normalized_values);
        d       = std::get<2>(normalized_values);

        normalize_transform.x = -start.x;
        normalize_transform.y = -start.y;
        normalize_transform.z = -start.z;
        normalize_transform.theta = std::get<3>(normalized_values);
    }

    void recompute()
    {
        normalize();
        compute_length();
        intermediate_pose_1 = update_dubins<fst>(start,fst_length,1.,climb,turn_radius);
        intermediate_pose_2 = update_dubins<snd>(intermediate_pose_1,snd_length,1.,climb,turn_radius);
    }

public:
    BaseDubins(const AircraftStats& stats, const Pose3D& _start, const Pose3D& _end)
    : Dubins(stats,_start,_end) {recompute();}

    BaseDubins(const AircraftStats& stats, double _alpha, double _beta, double _d, double _z)
    : Dubins(stats,{0.,0.,0.,_alpha},{_d,0.,_z,_beta}) {recompute();}

    BaseDubins(const AircraftStats& stats, const Pose3D& _start, const Pose3D& _end, double target_len, double tol)
    : Dubins(stats,_start,_end)
    {
        normalize();
        adjust_length(target_len,tol);

        fst_length = compute_fst_length(alpha,beta,d/turn_radius)*turn_radius;
        snd_length = compute_snd_length(alpha,beta,d/turn_radius)*turn_radius;
        trd_length = compute_trd_length(alpha,beta,d/turn_radius)*turn_radius;

        intermediate_pose_1 = follow_dubins<fst>(start,fst_length,1.,climb,turn_radius);
        intermediate_pose_2 = follow_dubins<snd>(intermediate_pose_1,snd_length,1.,climb,turn_radius);
    }

    Pose3D get_position(double len) const override
    {

#if DubinsFleetPlanner_ASSERTIONS > 0
        assert(len >= 0);
#endif

        if (len < fst_length)
        {
            return update_dubins<fst>(start,len,1.,climb,turn_radius);
        }
        else
        {
            len -= fst_length;
            if (len < snd_length)
            {
                return update_dubins<snd>(intermediate_pose_1,len,1.,climb,turn_radius);
            }
            else
            {
                len = std::min(len,trd_length);
                return update_dubins<trd>(intermediate_pose_2,len,1.,climb,turn_radius);
            }
        }
    }
};


typedef BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::LEFT> BaseDubinsLSL;       // LSL
typedef BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::RIGHT> BaseDubinsLSR;      // LSR
typedef BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::RIGHT> BaseDubinsRSR;     // RSR
typedef BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::LEFT> BaseDubinsRSL;      // RSL
typedef BaseDubins<DubinsMove::RIGHT,DubinsMove::LEFT,DubinsMove::RIGHT> BaseDubinsRLR;         // RLR
typedef BaseDubins<DubinsMove::LEFT,DubinsMove::RIGHT,DubinsMove::LEFT> BaseDubinsLRL;          // LRL
typedef BaseDubins<DubinsMove::STRAIGHT,DubinsMove::RIGHT,DubinsMove::STRAIGHT> BaseDubinsSRS;  // SRS
typedef BaseDubins<DubinsMove::STRAIGHT,DubinsMove::LEFT,DubinsMove::STRAIGHT> BaseDubinsSLS;   // SLS

typedef std::tuple<BaseDubinsLSL,BaseDubinsLSR,BaseDubinsRSR,BaseDubinsRSL,
        BaseDubinsRLR,BaseDubinsLRL,BaseDubinsSRS,BaseDubinsSLS> 
        AllBaseDubins;


AllBaseDubins list_possible_baseDubins(const AircraftStats& stats, const Pose3D& _start, const Pose3D& _end);
