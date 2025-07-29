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
#include "DubinsPrimitives.hpp"

#define _GENERATE_AWAIT(x) AWAIT_##x##_MS = ##(x)+3

enum DubinsMove {
    STRAIGHT = 0,
    LEFT = 1,
    RIGHT = 2
};

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
    Dubins(/* args */);
    ~Dubins();

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




};

template<DubinsMove fst, DubinsMove snd, DubinsMove trd>
class BaseDubins : public Dubins
{
private:
    static double normalized_length(double alpha, double beta, double d) [[gnu::const]];

    double alpha;   // Start orientation in normalized problem
    double beta;    // End orientation in normalized problem
    double d;       // Distance between endpoints in normalized problem
    Pose3D normalize_transform;     // Encode the transform from standard to normalized problem (add a shift then rotate) 

    /**
     * @brief  Compute the normalized equivalent of the given path planning problem.
     * 
     * That is, given the initial pose `start` and final pose `end`, compute the equivalent 
     * start at (0,0) oriented with angle `alpha` (radian) and end at (d,0) oriented with angle `beta`
     * 
     */
    void normalize()
    {
        normalize_transform.x = -start.x;
        normalize_transform.y = -start.y;
        normalize_transform.z = -start.z;

        double dx = end.x - start.x;
        double dy = end.y - start.y;

        double theta = std::atan2(dy,dx);

        normalize_transform.theta = theta;
        
        alpha   = start.theta - theta;
        beta    = end.theta - theta;
        d       = std::sqrt(dx*dx+dy*dy);
    }
};

