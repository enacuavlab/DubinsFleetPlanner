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

#include "inputParser.hpp"

typedef DubinsPP::InputParser::RowInfo RowInfo;
typedef DubinsPP::InputParser::CaseData CaseData;

template<typename T>
static inline std::from_chars_result from_stringview(const std::string_view& v, T& t)
{
    return std::from_chars(v.begin(), v.end(), t);
}

static inline void check_from_chars_result(const std::from_chars_result& res, const std::string_view& src)
{
    if (res.ec != std::errc())
    {
        
        std::cout << "Failed parsing:" << std::endl << src << std::endl;

        if (res.ec == std::errc::invalid_argument)
        {
            std::cout << " -> Not a number" << std::endl;
        }
        else if (rec.ec == std::errc::result_out_of_range)
        {
            stc::cout << " -> Number too big" << std::endl;
        }
    }

    assert(res.ec == std::errc());
}

template<typename T>
static void failable_from_stringview(const std::string_view& v, T& t)
{
    check_from_chars_result(from_stringview(v,t),v);
}

/**
 * @brief Check if the CSV row matches the expected header row
 * 
 * Reference CSV header: ac_id,start_x,start_y,start_z,start_theta,end_x,end_y,end_z,end_theta,airspeed,climb,turn_radius,dt
 * 
 * @param r The first CSV row of the file
 * @return true 
 * @return false 
 */
bool check_header(const CSVRow& r)
{
    std::cout << "Reading header..." << std::endl
        << "- Col 0 : " << r[0]     << " (" << r[0].compare ("ac_id")       << ")" << std::endl
        << "- Col 1 : " << r[1]     << " (" << r[1].compare ("start_x")     << ")" << std::endl
        << "- Col 2 : " << r[2]     << " (" << r[2].compare ("start_y")     << ")" << std::endl
        << "- Col 3 : " << r[3]     << " (" << r[3].compare ("start_z")     << ")" << std::endl
        << "- Col 4 : " << r[4]     << " (" << r[4].compare ("start_theta") << ")" << std::endl
        << "- Col 5 : " << r[5]     << " (" << r[5].compare ("end_x")       << ")" << std::endl
        << "- Col 6 : " << r[6]     << " (" << r[6].compare ("end_y")       << ")" << std::endl
        << "- Col 7 : " << r[7]     << " (" << r[7].compare ("end_z")       << ")" << std::endl
        << "- Col 8 : " << r[8]     << " (" << r[8].compare ("end_theta")   << ")" << std::endl
        << "- Col 9 : " << r[9]     << " (" << r[9].compare ("airspeed")    << ")" << std::endl
        << "- Col 10: " << r[10]    << " (" << r[10].compare("climb")       << ")" << std::endl
        << "- Col 11: " << r[11]    << " (" << r[11].compare("turn_radius") << ")" << std::endl
        << "- Col 12: " << r[12]    << " (" << r[12].compare("dt")          << ")" << std::endl
        << std::endl;


    bool test = (
        (r[0].compare ("ac_id")         == 0) &&
        (r[1].compare ("start_x")       == 0) &&
        (r[2].compare ("start_y")       == 0) &&
        (r[3].compare ("start_z")       == 0) &&
        (r[4].compare ("start_theta")   == 0) &&
        (r[5].compare ("end_x")         == 0) &&
        (r[6].compare ("end_y")         == 0) &&
        (r[7].compare ("end_z")         == 0) &&
        (r[8].compare ("end_theta")     == 0) &&
        (r[9].compare ("airspeed")      == 0) &&
        (r[10].compare("climb")         == 0) &&
        (r[11].compare("turn_radius")   == 0) &&
        ((r[12].compare("dt")  == 0) || (r[12].compare("dt\r")  == 0) || (r[12].compare("dt\n")  == 0) || (r[12].compare("dt\r\n")  == 0))

    );

    return test;
}

static RowInfo parse_row(const CSVRow& r)
{
    RowInfo o;

    std::from_chars_result res;

    failable_from_stringview(r[0] ,o.stats.id);

    failable_from_stringview(r[1] ,o.start.x);
    failable_from_stringview(r[2] ,o.start.y);
    failable_from_stringview(r[3] ,o.start.z);
    failable_from_stringview(r[4] ,o.start.theta);

    failable_from_stringview(r[5] ,o.end.x);
    failable_from_stringview(r[6] ,o.end.y);
    failable_from_stringview(r[7] ,o.end.z);
    failable_from_stringview(r[8] ,o.end.theta);

    failable_from_stringview(r[9] ,o.stats.airspeed);
    failable_from_stringview(r[10],o.stats.climb);
    failable_from_stringview(r[11],o.stats.turn_radius);

    failable_from_stringview(r[12],o.dt);

    return o;
}

static CaseData tranpose_data(const std::vector<RowInfo>& rows)
{
    uint N = rows.size();

    std::vector<Pose3D> starts(N);
    std::vector<Pose3D> ends(N);
    std::vector<AircraftStats> stats(N);
    std::vector<double> dt(N-1);

    for(uint i = 0; i < N; i++)
    {
        if (i > 0)
        {
            dt[i-1] = rows[i].dt;
        }

        starts[i]   = rows[i].start;
        ends[i]     = rows[i].end;
        stats[i]    = rows[i].stats;
    }

    return std::make_tuple(starts,ends,stats,dt);
}

CaseData DubinsPP::InputParser::parse_data_csv(std::istream& stream)
{
    bool first = true;
    std::vector<RowInfo> parsed_rows;

    for(auto &row: CSVRange(stream))
    {
        if (first)
        {
            if (!(check_header(*row)))
            {
                std::cout << std::endl << "========== WRONG HEADER! ==========" << std::endl << "Exiting now..." << std::endl; 
                exit(1);
            }
            first = false;
            continue;
        }

        parsed_rows.push_back(parse_row(row));
    }

    return tranpose_data(parsed_rows);

}