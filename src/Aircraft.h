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




typedef struct {
    double x; // X position
    double y; // Y position
    double a; // XY Orientation (in radiants, 0 is pure X, pi/2 is pure Y)
    double z; // Z position
} Pose3D;

typedef struct {
    double airspeed;     // Constant airspeed, in m/s
    double turn_radius;  // Minimal turn radius, in m
    double climb;        // Climb rate, in m/s
} AircraftStats;