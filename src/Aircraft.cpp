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

#include "Aircraft.h"

void move_straight(Pose3D& pose, double duration, double speed, double climb_rate)
{
    double x_dir = std::cos(pose.theta);
    double y_dir = std::sin(pose.theta);

    pose.x += x_dir*duration*speed;
    pose.y += y_dir*duration*speed;
    pose.z += climb_rate*duration;
}

[[gnu::const]]
Pose3D move_straight(const Pose3D& pose, double duration, double speed, double climb_rate)
{
    Pose3D output(pose);
    move_straight(output,duration,speed,climb_rate);
    return output;
}

void turn_left(Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    double len = speed*duration;    // Distance travelled
    double alpha = len/turn_radius; // Angle travelled (in radian)

    pose.z += climb_rate*duration;
}

[[gnu::const]]
Pose3D turn_left(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    Pose3D output(pose);
    turn_left(output,duration,speed,climb_rate,turn_radius);
    return output;
}

void turn_right(Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    double len = speed*duration;    // Distance travelled
    double alpha = len/turn_radius; // Angle travelled (in radian)

    pose.z += climb_rate*duration;
}

[[gnu::const]]
Pose3D turn_right(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius)
{
    Pose3D output(pose);
    turn_right(output,duration,speed,climb_rate,turn_radius);
    return output;
}