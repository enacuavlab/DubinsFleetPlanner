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

#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <memory>

#include <boost/container_hash/hash.hpp>

#include "ProjectHeader.h"

#include "utils.hpp"
#include "Aircraft.h"
#include "Primitives.hpp"
#include "Fitting.hpp"

const uint DubinsDistDefaultRec = 6;

class Dubins
{
protected:
    Pose3D start;   // Start pose (x,y,z, XY orientation)
    Pose3D end;     // End pose (x,y,z, XY orientation)

    double climb;       // Z Climb rate, in [alt]/m on the ground
    double turn_radius; // XY Turn radius, in m
    double length;      // Path length, in m. Is NAN if the path is not possible.

    bool valid = false; // Whether the desired path is possible or not

    /**
     * @brief Recompute the different values deduced from the problem if the start, end or control ever change
     */
    virtual void recompute() = 0;

    /**
     * @brief Compute and return the path length.
     * 
     * @return double Path length
     */
    virtual double _compute_length() = 0;

public:
    /**
     * @brief Ensure that the path ends where it is supposed to
     * 
     * This updates the 'valid' attribute, setting it to false if the end is not correct 
     * 
     * @return true The path ends correctly
     * @return false Either the path is already invalid, or it does not ends correctly
     */
    bool check_valid_end()
    {
        bool out = false;
        if (valid)
        {
            Pose3D end_check = get_position(length);
            out = pose_dist(end,end_check) < DubinsFleetPlanner_PRECISION;
            valid &= out;
        }
        return out;
    }

    /**
     * @brief Same as `_compute_length`, but also set the `length` and `valid` attributes
     * 
     * @return double 
     */
    double compute_length()
    {
        length  = _compute_length(); 
        valid   = (std::isfinite(length)) && (length >= 0);
        return length;
    }

    

    /****** Constructors ******/

    Dubins(double _climb, double _turn_radius, const Pose3D& _start, const Pose3D& _end)
    : start(_start), end(_end), climb(_climb), turn_radius(_turn_radius) {}

    /****** Setters and getters ******/

    // virtual constexpr const std::string& get_type_name() const = 0;
    virtual const std::string get_type_abbr(bool) const = 0;
    virtual const std::vector<DubinsMove> get_all_sections() const = 0;

    virtual size_t type_hash() const = 0;

    void set_start(const Pose3D& _start) {start = _start; recompute();}
    Pose3D get_start() const {return start;}

    void set_end(const Pose3D& _end) {end = _end; recompute();}
    Pose3D get_end() const {return end;}

    void set_climb(double _c) {climb = _c; recompute();}
    double get_climb() const {return climb;}

    void set_turn_radius(double _r) {turn_radius = _r; recompute();}
    double get_turn_radius() const {return turn_radius;}

    double get_length() const {return length;}
    double get_duration(double speed) const {return length/speed;}
    bool is_valid() const {return valid;}

    virtual Pose3D get_position(double len) const = 0;
    Pose3D get_position(double time, double speed) const
    {

#if DubinsFleetPlanner_ASSERTIONS > 0
        assert(speed > 0);
#endif

        return get_position(time*speed);
    }

    template<unsigned samples>
    std::array<Pose3D,samples> get_positions(const std::array<double,samples>& lens, [[maybe_unused]] bool sorted=false) const
    {
        std::array<Pose3D,samples> output;
        for(unsigned i = 0; i < samples; i++)
        {
            output[i] = get_position(lens[i]);
        }

        return output;
    }

    virtual const std::vector<Pose3D> get_positions(std::vector<double>& lens, [[maybe_unused]] bool sorted=false) const
    {
        std::vector<Pose3D> output;
        for(const double& l : lens)
        {
            output.push_back(get_position(l));
        }
        return output;
    }

    const std::vector<Pose3D> get_positions(std::vector<double>& times, double speed, bool sorted=false) const
    {

#if DubinsFleetPlanner_ASSERTIONS > 0
        assert(speed > 0);
#endif

        std::vector<double> lens(times);
        for(double& e : lens)
        {
            e *= speed;
        }
        return get_positions(lens,sorted);
    }

    template<unsigned samples>
    std::array<Pose3D,samples> get_position(const std::array<double,samples>& times, double speed, bool sorted=false) const
    {
        return get_positions<samples>(times*speed,sorted);
    }

    /**
     * @brief Get the location (in length coordinate) of the junction points between shapes
     * 
     * @return std::vector<double> 
     */
    virtual std::vector<double> get_junction_locs() const = 0;

    /**
     * @brief Get the location (in length coordinate) of the endpoints, ie start, junctions and end
     * 
     * @return std::vector<double> 
     */
    std::vector<double> get_endpoints_locs() const
    {
        std::vector<double> output{0.};

        std::vector<double> junctions = get_junction_locs();
        for(double l: junctions)
        {
            output.push_back(l);
        }

        output.push_back(get_length());
        return output;
    }

    /**
     * @brief Get the poses at the junctions between shapes
     * 
     * @return std::vector<Pose3D> 
     */
    virtual std::vector<Pose3D> get_junction_points() const = 0;

    /**
     * @brief Get the poses of the endpoints, ie start, junctions and end
     * 
     * @return std::vector<Pose3D> 
     */
    std::vector<Pose3D> get_endpoints() const
    {
        std::vector<Pose3D> output;
        output.push_back(get_start());

        std::vector<Pose3D> junctions = get_junction_points();
        for(Pose3D l: junctions)
        {
            output.push_back(l);
        }

        output.push_back(get_end());
        return output;
    }

    virtual DubinsMove get_section_type(double loc) const = 0;


    /**
     * @brief Compute the first point such that 'this' and 'that' breach their minimal separation,
     * or the minimal distance between the two proving that they are separated
     * 
     * 
     * @param other A second Dubins path
     * @param this_speed    XY Speed along `this` path (in [L]/s)
     * @param other_speed   XY Speed along `other` path (in [L]/s)
     * @param duration Duration for which to look for conflicts (in s)
     * @param min_dist The minimal distance required for ensuring separation (in [L])
     * @param tol      Solver tolerance used when looking for the minimal distance
     * @param hostart  Optional value for hotstarting the search: a place where the minimal separation might fail
     * @return A pair containing the location and value of either:
     *                  - The first trajectory part breaching the minimal distance
     *                  - The global minimal distance between the two
     */
    std::pair<double,double> XY_distance_to(const Dubins& other, double this_speed, double other_speed, 
        double duration, double min_dist, double tol, std::optional<double> hostart = std::nullopt, uint rec=DubinsDistDefaultRec) const;


    /**
     * @brief A static version of `XY_distance_to`
     *
     */
    static std::pair<double,double> compute_XY_distance(const Dubins& first, const Dubins& second, double first_speed, double second_speed, 
        double duration, double min_dist,
        double tol=DubinsFleetPlanner_PRECISION, 
        std::optional<double> hotstart = std::nullopt,
        uint rec=DubinsDistDefaultRec)
    {
        return first.XY_distance_to(second,first_speed,second_speed,duration,min_dist,tol,hotstart,rec);
    }

    /**
     * @brief Generic type for distance function
     * 
     * The arguments are:
     * First path, second path, first speed, second speed, duration, minimal distance, computation precision, optional hot start, heuristic recursion
     * 
     * Returns the location and value of the minimal distance between trajectories
     */
    typedef std::pair<double,double> (*DubinsDistanceFunction)(const Dubins&, const Dubins&, 
        double, double, double, double, double, std::optional<double>, uint);

    /**
     * @brief Check whether `this` and `other` are horitzontaly separated
     * 
     * 
     * @param other A second Dubins path
     * @param this_speed    XY Speed along `this` path (in [L]/s)
     * @param other_speed   XY Speed along `other` path (in [L]/s)
     * @param duration Duration for which to look for conflicts (in s)
     * @param min_dist The minimal distance required for ensuring separation (in [L])
     * @param tol      Solver tolerance used when looking for the minimal distance
     * @param rec      Depth of the recursive call to the fast but inexact geometric approximation
     * @return true  The two trajectories are well separated
     * @return false The two trajectories are not separated
     */
    bool is_XY_separated_from(const Dubins &other, double this_speed, double other_speed,
                              double duration, double min_dist, 
                              double tol = DubinsFleetPlanner_PRECISION,
                              uint rec = DubinsDistDefaultRec) const;

    /**
     * @brief A static version of `is_XY_separated_from`
     *
     */
    static bool are_XY_separated(const Dubins& first, const Dubins& second, double first_speed, double second_speed, 
        double duration, double min_dist,
        double tol=DubinsFleetPlanner_PRECISION,
        uint rec = DubinsDistDefaultRec)
    {
        return first.is_XY_separated_from(second,first_speed,second_speed,duration,min_dist,tol,rec);
    }

    /**
     * @brief Check wether two Dubins paths are separated by using sampling
     * 
     * @param first         First path
     * @param second        Second path
     * @param first_speed   First path speed
     * @param second_speed  Second path speed
     * @param duration      Duration over which to check for separation
     * @param min_dist      Minimal distance required
     * @param tol           Distance between two successive time samples
     * @param rec           UNUSED, required for compatibility with other functions
     * @return true         The two trajectories are separated
     * @return false        There is a conflict
     */
    static bool are_XY_separated_sampling(const Dubins& first, const Dubins& second, double first_speed, double second_speed, 
        double duration, double min_dist,
        double tol, [[maybe_unused]] uint rec)
    {
        double t = 0.;
        while (t < duration)
        {
            Pose3D p1 = first.get_position(t*first_speed);
            Pose3D p2 = second.get_position(t*second_speed);

            double dx = p1.x - p2.x;
            double dy = p1.y - p2.y;

            if (dx*dx + dy*dy < min_dist*min_dist)
            {
                return false;
            }

            t += tol;
        }

        return true;
    }

    /**
     * @brief Generic type for separation function
     * 
     * The arguments are:
     * First path, second path, first speed, second speed, duration, minimal distance, computation precision, heuristic recursion
     */
    typedef bool (*DubinsSeparationFunction)(const Dubins&, const Dubins&, double, double, double, double, double, uint);

    /**
     * @brief Generic type for path generation function (minimal turn radius)
     * 
     * The arguments are:
     * Climb ratio, turn radius, start pose, end pose
     * 
     * Returns the list of valid paths as unique_ptr
     */
    typedef std::vector<std::unique_ptr<Dubins>> (*PathGeneratorFunction)(double, double, const Pose3D&, const Pose3D&);

    /**
     * @brief Generic type for fitted path generation function
     * 
     * The arguments are:
     * Climb ratio, turn radius, start pose, end pose, target length, length tolerance
     * 
     * Returns the list of valid paths as unique_ptr
     */
    typedef std::vector<std::unique_ptr<Dubins>> (*FittedPathGeneratorFunction)(double, double, const Pose3D&, const Pose3D&, double, double);
};