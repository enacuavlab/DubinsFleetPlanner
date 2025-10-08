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

#include <format>

#include "Dubins.hpp"
#include "BaseDubins.hpp"

template<DubinsMove beginMove ,DubinsMove endMove>
class ExtendedDubins : public Dubins
{
private:
    const std::unique_ptr<Dubins> underlying;
    double begin_length,end_length;
    Pose3D begin_junction,end_junction;

    /**
     * @brief Recompute the different values deduced from the problem if the start, end or control ever change
     */
    void recompute()
    {
        begin_junction = follow_dubins<beginMove>(start,begin_length,1.,climb,turn_radius);
        end_junction = follow_dubins<endMove>(end,-end_length,1.,climb,turn_radius);
        underlying->set_start(begin_junction);
        length = _compute_length();
        valid = underlying->is_valid();
    }

    /**
     * @brief Compute and return the path length.
     * 
     * @return double Path length
     */
    double _compute_length()
    {
        length = begin_length + underlying->compute_length() + end_length;
        return length;
    }

public:

    /****** Constructors ******/

    ExtendedDubins(double _climb, double _turn_radius, 
        const Pose3D& _start, const Pose3D& _end, double _begin_len, double _end_len,
        std::unique_ptr<Dubins>& _underlying)
    : Dubins(_climb, _turn_radius, _start, _end),
        begin_length(_begin_len), end_length(_end_len),
        underlying(std::move(_underlying))
    {
        recompute();
    }

    ExtendedDubins(ExtendedDubins<beginMove,endMove>& other) :
        Dubins(other._climb, other._turn_radius, other._start, other._end),
        begin_length(other._begin_len), end_length(other._end_len),
        underlying(std::move(other._underlying)),
        begin_junction(other.begin_junction),
        end_junction(other.end_junction)
        {
            this->length = other.get_length();
        }


    /****** Setters and getters ******/

    const std::string get_type_abbr() const
    {
        std::string output;
        if (std::abs(begin_length) > DubinsFleetPlanner_PRECISION)
        {
            output += std::string(1,DubinsMoveNames[beginMove][0]) + std::format(" {:.2f} -",begin_length);
        }

        output += underlying->get_type_abbr();

        if (std::abs(end_length) > DubinsFleetPlanner_PRECISION)
        {
            output += std::string("- ") + std::string(1,DubinsMoveNames[endMove][0]) + std::format(" {:.2f}",end_length);
        }

        return output;
    }

    const std::vector<DubinsMove> get_all_sections() const
    {
        std::vector<DubinsMove> output(underlying->get_all_sections());
        output.insert(output.begin(),beginMove);
        output.push_back(endMove);
        return output;
    }


    Pose3D get_position(double len) const
    {
        if (len < begin_length)
        {
            return follow_dubins<beginMove>(start,len,1.,climb,turn_radius);
        }
        else
        {
            len -= begin_length;

            if (len < underlying->get_length())
            {
                return underlying->get_position(len);
            }
            else
            {
                return follow_dubins<endMove>(underlying->get_end(),len - underlying->get_length(), 1., climb, turn_radius);
            }
        }
    }

    /**
     * @brief Get the location (in length coordinate) of the junction points between shapes
     * 
     * @return std::vector<double> 
     */
    std::vector<double> get_junction_locs() const
    {
        std::vector<double> output;
        output.push_back(begin_length);
        for(double l : underlying->get_junction_locs())
        {
            output.push_back(l+begin_length);
        }
        output.push_back(begin_length+underlying->get_length());

        return output;
    }

    std::vector<Pose3D> get_junction_points() const
    {
        std::vector<Pose3D> output;
        output.push_back(underlying->get_start());

        std::vector<Pose3D> underlying_points = underlying->get_junction_points();
        output.insert(output.end(),underlying_points.begin(),underlying_points.end());

        output.push_back(underlying->get_end());

        return output;
    }

    DubinsMove get_section_type(double loc) const
    {
        if (loc < begin_length)
        {
            return beginMove;
        }
        else
        {
            loc -= begin_length;
            if (loc < underlying->get_length())
            {
                return underlying->get_section_type(loc);
            }
            else
            {
                return endMove;
            }
        }
    }

};

// ---------- Generator functions ---------- //

// ----- Generic templates ----- //

template <DubinsMove beginMove, DubinsMove endMove, Dubins::PathGeneratorFunction generator_function>
std::vector<std::unique_ptr<Dubins>> generate_extended_dubins(
    const Pose3D& start, const Pose3D& end, std::vector<double> start_lens, std::vector<double> end_lens,
    double climb, double turn_radius)
{
    std::vector<std::unique_ptr<Dubins>> output;

    for(double sl : start_lens)
    {
        // Only keep null values when move is straight (cut redundant parts)
        if (beginMove != STRAIGHT)
        {
            if (std::abs(sl) < DubinsFleetPlanner_PRECISION)
            {
                continue;
            }
        }

        Pose3D shift_start = follow_dubins<beginMove>(start,sl,1.,climb,turn_radius);

        for(double el : end_lens)
        {
            // Only keep null values when move is straight (cut redundant parts)
            if (endMove != STRAIGHT)
            {
                if (std::abs(el) < DubinsFleetPlanner_PRECISION)
                {
                    continue;
                }
            }

            Pose3D shift_end = follow_dubins<endMove>(end,-el,1.,climb,turn_radius);

            auto generated = generator_function(climb, turn_radius, shift_start, shift_end);

            for(auto& b : generated)
            {
                output.push_back(std::make_unique<ExtendedDubins<beginMove,endMove>>(climb,turn_radius,start,end,sl,el,b));
            }
        }
    }

    return output;
}

/**
 * @brief Given an arbitrary fitted path generating function, extend the generated paths
 * 
 * @tparam beginMove    Type of the begin extension
 * @tparam endMove      Type of the end extension
 * @tparam gen_F_T      Generic type for the path generator function
 * @param generator_function    Path generator function
 * @param start 
 * @param end 
 * @param start_lens 
 * @param end_lens 
 * @param _climb 
 * @param _turn_radius 
 * @param target_len 
 * @param tol 
 * @return std::vector<std::unique_ptr<Dubins>> 
 */
template <DubinsMove beginMove, DubinsMove endMove, Dubins::FittedPathGeneratorFunction generator_function>
std::vector<std::unique_ptr<Dubins>> generate_extended_fitted_dubins(
    const Pose3D& start, const Pose3D& end, std::vector<double> start_lens, std::vector<double> end_lens,
    double _climb, double _turn_radius, double target_len, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output;

    for(double sl : start_lens)
    {

        Pose3D shift_start = follow_dubins<beginMove>(start,sl,1.,_climb, _turn_radius);

        for(double el : end_lens)
        {
            Pose3D shift_end = follow_dubins<endMove>(end,-el,1.,_climb, _turn_radius);

            if (target_len - sl - el < 0.)
            {
                continue;
            }

            auto generated = generator_function(_climb, _turn_radius, shift_start, shift_end, target_len-sl-el,tol);

            for(auto& b : generated)
            {
                output.push_back(std::make_unique<ExtendedDubins<beginMove,endMove>>(
                    _climb, _turn_radius,start,end,sl,el,b)
                );
            }        
        }
    }

    return output;
}

// ----- Path generation functions ----- //

/**
 * @brief Generate extended Dubins path from base ones
 * 
 * The Extended Dubins paths are made by adding set paths after the start and before the end,
 * thus generating a new Dubins problem which is solved normally. 
 * 
 * @param start     Start pose
 * @param end       End pose
 * @param start_lens    Lengths for the added initial segment
 * @param end_lens      Lengths for the added final segment
 * @param climb         Climb ratio
 * @param turn_radius   Minimal turn radius
 * @return std::vector<std::unique_ptr<Dubins>>  List of generated paths
 */
std::vector<std::unique_ptr<Dubins>> generate_all_extended_from_baseDubins(const Pose3D& start, const Pose3D& end, 
    std::vector<double> start_lens, std::vector<double> end_lens,
    double climb, double turn_radius);

/**
 * @brief Generate fitted extended Dubins path from base ones
 * 
 * The Extended Dubins paths are made by adding set paths after the start and before the end,
 * thus generating a new Dubins problem which is solved normally. 
 * 
 * @param start     Start pose
 * @param end       End pose
 * @param start_lens    Lengths for the added initial segment
 * @param end_lens      Lengths for the added final segment
 * @param climb         Climb ratio
 * @param turn_radius   Minimal turn radius
 * @param target_len    Target length
 * @param tol           Precision for target length
 * @return std::vector<std::unique_ptr<Dubins>>     List of generated paths
 */
std::vector<std::unique_ptr<Dubins>> generate_all_extended_from_fitted_baseDubins(const Pose3D& start, const Pose3D& end, 
    std::vector<double> start_lens, std::vector<double> end_lens,
    double climb, double turn_radius, double target_len, double tol);