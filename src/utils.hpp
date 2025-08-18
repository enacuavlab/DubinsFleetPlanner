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
#include <tuple>

// -------------------- General maths -------------------- //

/**
 * @brief Reduce an angle in radian to [0,2*Pi]
 * 
 * @param x Input angle
 * @return double Reduced equivalent value in [0,2*Pi]
 */
double mod_2pi(double x);

/**
 * @brief Reduce an angle to its central form, i.e. into the interval [-Pi,Pi]
 * 
 * @param x Angle
 * @return double Equivalent to x in [-pi,pi]
 */
double central_angle(double x);

// -------------------- Pose and mouvements -------------------- //

typedef struct {
    double x    ; // X position
    double y    ; // Y position
    double z    ; // Z position
    double theta; // XY Orientation (in radiants, 0 is pure X, pi/2 is pure Y)
} Pose3D;

/**
 * @brief Apply a straight (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m 
 */
void move_straight(Pose3D* pose, double duration, double speed, double climb_rate);

/**
 * @brief Apply a straight (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m 
 * @return Pose3D       Resulting position
 */
Pose3D move_straight(const Pose3D& pose, double duration, double speed, double climb_rate);

/**
 * @brief Apply a left turn (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m
 */
void turn_left(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Apply a left turn (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m 
 * @return Pose3D       Resulting position
 */
Pose3D turn_left(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Apply a right turn (XY) movement to the Pose
 * 
 * This function modifies the given pose
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m
 */
void turn_right(Pose3D* pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief Apply a right turn (XY) movement to the Pose
 * 
 * This function returns a fresh pose without modifying the starting one
 * 
 * @param pose          Starting pose 
 * @param duration      Movement duration (s)
 * @param speed         XY Speed (positive, m/s)
 * @param climb_rate    Z Speed ([alt]/s)
 * @param turn_radius   Turn radius, in m 
 * @return Pose3D       Resulting position
 */
Pose3D turn_right(const Pose3D& pose, double duration, double speed, double climb_rate, double turn_radius);

/**
 * @brief  Compute the normalized equivalent of the given path planning problem.
 * 
 * That is, given the initial pose `start` and final pose `end`, compute the equivalent 
 * start at (0,0) oriented with angle `alpha` (radian) and end at (d,0) oriented with angle `beta`
 * 
 * @param start     Initial pose
 * @param end       Ending pose
 * @return std::tuple<double,double,double,double> (alpha, beta, d, theta), with theta being the rotation needed
 * to align the `end` on the X axis
 */
std::tuple<double,double,double,double> normalize_poses(const Pose3D& start, const Pose3D& end);

/**
 * @brief Compute the 3D Euclidean distance between two poses
 * 
 * @param p1 First pose
 * @param p2 Second pose
 * @return double 3D Euclidean distance between the points
 */
double pose_dist(const Pose3D& p1, const Pose3D& p2);

/**
 * @brief Compute the 2D Euclidean distance between two poses, using the XY components
 * 
 * @param p1 First pose
 * @param p2 Second pose
 * @return double 2D Euclidean distance between the XY projected points
 */
double pose_dist_XY(const Pose3D& p1, const Pose3D& p2);