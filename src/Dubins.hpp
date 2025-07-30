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

#include <vector>
#include <cmath>

#include "Aircraft.h"
#include "Primitives.hpp"
#include "Fitting.hpp"

#define _GENERATE_AWAIT(x) AWAIT_##x##_MS = ##(x)+3

class Dubins
{
protected:
    Pose3D start;
    Pose3D end;

    double climb;
    double turn_radius;
    double length;

    bool valid = false;

    virtual void recompute() = 0;
    virtual double compute_length() = 0;

public:
    Dubins(const AircraftStats& stats, const Pose3D& _start, const Pose3D& _end)
    : climb(stats.climb), turn_radius(stats.turn_radius), start(_start), end(_end)
    {
        recompute();
    }

    /****** Setters and getters ******/

    void set_start(const Pose3D& _start) {start = _start; recompute();}
    Pose3D get_start() {return start;}

    void set_end(const Pose3D& _end) {end = _end; recompute();}
    Pose3D get_end() {return end;}

    void set_climb(double _c) {climb = _c; recompute();}
    double get_climb() {return climb;}

    void set_turn_radius(double _r) {turn_radius = _r; recompute();}
    double get_turn_radius() {return turn_radius;}

    double get_length() {return length;}

    virtual Pose3D get_position(double len) = 0;
    Pose3D get_position(double time, double speed) {return get_position(time*speed);}

    virtual std::vector<Pose3D> get_positions(const std::vector<double>& lens) = 0;
    std::vector<Pose3D> get_positions(const std::vector<double>& times, double speed)
    {
        std::vector<double> lens(times);
        for(double& e : lens)
        {
            e *= speed;
        }
        return get_positions(lens);
    }
};
