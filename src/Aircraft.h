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

#include <cmath>

typedef struct {
    double x    ; // X position
    double y    ; // Y position
    double theta; // XY Orientation (in radiants, 0 is pure X, pi/2 is pure Y)
    double z    ; // Z position
} Pose3D;

typedef struct {
    double airspeed;     // Constant airspeed, in m/s
    double turn_radius;  // Minimal turn radius, in m
    double climb;        // Climb rate, in [alt]/s (with [alt] some unit for altitude; meters above sea level, feets above ground...)
} AircraftStats;


/**
 * @brief Apply a straight (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose      Starting pose 
 * @param duration  Movement duration
 * @param speed     XY Speed (positive, m/s)
 * @param climb_rate Z Speed ([alt]/s)
 */
void move_straight(Pose3D& pose, double duration, double speed, double climb_rate);

/**
 * @brief Apply a straight (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose      Starting pose 
 * @param duration  Movement duration (s)
 * @param speed     XY Speed (positive, m/s)
 * @param climb_rate Z Speed ([alt]/s)
 * @return Pose3D   Resulting position
 */
Pose3D move_straight(const Pose3D& pose, double duration, double speed, double climb_rate) [[gnu::const]];

/**
 * @brief Apply a left turn (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose      Starting pose 
 * @param duration  Movement duration (s)
 * @param speed     XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m
 */
void turn_left(Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Apply a left turn (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose      Starting pose 
 * @param duration  Movement duration (s)
 * @param speed     XY Speed (positive, m/s)
 * @param climb_rate Z Speed ([alt]/s)
 * @return Pose3D   Resulting position
 */
Pose3D turn_left(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius) [[gnu::const]];

/**
 * @brief Apply a right turn (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose      Starting pose 
 * @param duration  Movement duration (s)
 * @param speed     XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m
 */
void turn_right(Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Apply a right turn (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose      Starting pose 
 * @param duration  Movement duration (s)
 * @param speed     XY Speed (positive, m/s)
 * @param climb_rate Z Speed ([alt]/s)
 * @return Pose3D   Resulting position
 */
Pose3D turn_right(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius) [[gnu::const]];