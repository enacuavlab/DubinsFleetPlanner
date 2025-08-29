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
#include <string>
#include <memory>
#include <vector>
#include <tuple>

#include "csv_reader.hpp"

#include "ProjectHeader.h"

#include "Aircraft.h"
#include "utils.hpp"
#include "Dubins.hpp"


namespace DubinsPP
{
    namespace InputParser
    {
        // Storage for case data has shown in a CSV line
        typedef struct 
        {
            Pose3D start;
            Pose3D end;
            AircraftStats stats;
            double dt;
        } RowInfo;

        // Transposed type of std::vector<RowInfo>
        typedef std::tuple<std::vector<Pose3D>,std::vector<Pose3D>,std::vector<AircraftStats>,std::vector<double>> CaseData;

        /**
         * @brief Parse an input stream reading a CSV formatted for a path planning data into the matching data
         * 
         * The first line (header) should exactly as follow:
         * ac_id,start_x,start_y,start_z,start_theta,end_x,end_y,end_z,end_theta,airspeed,climb,turn_radius,dt
         * 
         * The expected types are:
         * int (positive),float,float,float,float   ,float,float,float,float    ,float (positive),float,float (positive),float
         * 
         * The following data rows should match this format for correct parsing
         * 
         * @return CaseData 
         */
        CaseData parse_data_csv(std::istream&);

    } // namespace InputParser

    namespace OutputPrinter
    {
        void print_paths_as_JSON(std::ostream&, 
            const std::vector<std::unique_ptr<Dubins>>& paths,
            const std::vector<AircraftStats>& stats,
            double wind_x, double wind_y);

        void print_paths_as_CSV(std::ostream&, 
            const std::vector<std::unique_ptr<Dubins>>& paths,
            const std::vector<AircraftStats>& stats,
            double wind_x, double wind_y, uint samples);
    } // namespace OutputPrinter
    
} // namespace DubinsPP