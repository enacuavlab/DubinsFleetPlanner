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

#include <assert.h>
#include <charconv>
#include <iostream>

#include "ProjectHeader.h"

#include "Aircraft.h"
#include "utils.hpp"

#include "csv_reader/csv_reader.hpp"

namespace DubinsPP
{
    namespace InputParser
    {
        typedef struct 
        {
            Pose3D start;
            Pose3D end;
            AircraftStats stats;
            double dt;
        } RowInfo;

        typedef std::tuple<std::vector<Pose3D>,std::vector<Pose3D>,std::vector<AircraftStats>,std::vector<double>> CaseData;

        CaseData parse_data_csv(std::ifstream&);
    }
}