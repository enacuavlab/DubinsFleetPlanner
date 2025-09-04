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

#include "ioUtils.hpp"

// ---------------------------------------- Misc function ---------------------------------------- //

static double compute_plan_duration(
    const std::vector<std::unique_ptr<Dubins>>& paths,
    const std::vector<AircraftStats>& stats)
{
    assert(paths.size() == stats.size());
    uint N = paths.size();

    double max_time = 0;

    for(uint i = 0; i < N; i++)
    {
        double time = paths[i]->get_length()/stats[i].airspeed;
        max_time = std::max(time,max_time);
    }

    return max_time;
}

// ---------------------------------------- Input handling ---------------------------------------- //

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
        else if (res.ec == std::errc::result_out_of_range)
        {
            std::cout << " -> Number too big" << std::endl;
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
            if (!(check_header(row)))
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

// ---------------------------------------- Output handling ---------------------------------------- //

template<typename T>
void print_field(std::ostream& s, const std::string& name, const T& value, bool trailing, uint indent_lvl)
{
    for(uint i = 0; i < indent_lvl; i++)
    {
        s << "\t";
    }

    s << "\"" << name << "\": " << value;

    if (trailing)
    {
        s << "," << std::endl;
    }
    else
    {
        s << std::endl;
    }
}

template<> 
void print_field<Pose3D>(std::ostream& s, const std::string& name, const Pose3D& value, bool trailing, uint indent_lvl)
{
    for(uint i = 0; i < indent_lvl; i++)
    {
        s << "\t";
    }

    s   << "\"" << name << "\": "
        << "{\"x\": " << value.x 
        << " ,\"y\": " << value.y 
        << " ,\"z\": " << value.z 
        << " ,\"theta\": " << value.theta 
        << " }";

    if (trailing)
    {
        s << "," << std::endl;
    }
    else
    {
        s << std::endl;
    }

}

template<> 
void print_field<std::string>(std::ostream& s, const std::string& name, const std::string& value, bool trailing, uint indent_lvl)
{
    for(uint i = 0; i < indent_lvl; i++)
    {
        s << "\t";
    }

    s << "\"" << name << "\": \"" << value << "\"";

    if (trailing)
    {
        s << "," << std::endl;
    }
    else
    {
        s << std::endl;
    }

}


template<DubinsMove m>
static inline void print_pathShape_fields(std::ostream& s, PathShape<m> p, uint indent_lvl=6)
{
    print_field(s,"x",p.x,true,indent_lvl);
    print_field(s,"y",p.y,true,indent_lvl);
    print_field(s,"z",p.z,true,indent_lvl);
    print_field(s,"p1",p.p1,true,indent_lvl);
    print_field(s,"p2",p.p2,true,indent_lvl);
    print_field(s,"p3",p.p3,true,indent_lvl);
    print_field(s,"p4",p.p4,true,indent_lvl);
    print_field(s,"type",get_DubinsMove_name(m),false,indent_lvl);
}


void print_trajectory(std::ostream& s, 
    const std::unique_ptr<Dubins>& path,
    const AircraftStats& stats,
    double wind_x, double wind_y, double duration)
{
    s << "\t\t{" << std::endl;
    {
        s << "\t\t\t\"stats\": {" << std::endl;
        {
            print_field(s,"id",stats.id,true,4);
            print_field(s,"airspeed",stats.airspeed,true,4);
            print_field(s,"climb",stats.climb,true,4);
            print_field(s,"turn_radius",stats.turn_radius,false,4);
        }
        s << "\t\t\t}," << std::endl;

        s << "\t\t\t\"path\": {" << std::endl;
        {
            print_field(s,"total_length",path->get_length(),true,4);
            print_field(s,"sections_count",path->get_all_sections().size(),true,4);
            print_field(s,"start",path->get_start(),true,4);
            print_field(s,"end",path->get_end(),true,4);

            s << "\t\t\t\t\"sections\":[";
            {
                std::vector<double> endpoints_locs = path->get_endpoints_locs();
                std::vector<Pose3D> endpoints = path->get_endpoints();
                uint sections_N = endpoints.size()-1;

                for(uint i = 0; i < sections_N; i++)
                {
                    if (i > 0) {s << ",";}
                    
                    s << std::endl;

                    double sec_len = endpoints_locs[i+1]-endpoints_locs[i];
                    double sec_middle = (endpoints_locs[i+1]+endpoints_locs[i])/2;
                    DubinsMove sec_type = path->get_section_type(sec_middle);
                    Pose3D& sec_start = endpoints[i];
                    Pose3D& sec_end   = endpoints[i+1];
                    
                    s << "\t\t\t\t\t{" << std::endl;
                    {
                        print_field(s,"length",sec_len,true,6);
                        switch (sec_type)
                        {
                        case STRAIGHT:
                            print_pathShape_fields<STRAIGHT>(s,
                                compute_params<STRAIGHT>(sec_start,sec_end,stats.airspeed,path->get_turn_radius(),path->get_climb()),6);
                            break;

                        case LEFT:
                            print_pathShape_fields<LEFT>(s,
                                compute_params<LEFT>(sec_start,sec_end,stats.airspeed,path->get_turn_radius(),path->get_climb()),6);
                            break;

                        case RIGHT:
                            print_pathShape_fields<RIGHT>(s,
                                compute_params<RIGHT>(sec_start,sec_end,stats.airspeed,path->get_turn_radius(),path->get_climb()),6);
                            break;
                        
                        default:
                            // Unreachable
                            exit(EXIT_FAILURE);
                        }
                    }
                    s << "\t\t\t\t\t}" << std::endl;

                }
            }
            s << "\t\t\t\t]";
        }
        s << "\t\t\t}" << std::endl;
    }

    s << "\t\t}" << std::endl;
}

void DubinsPP::OutputPrinter::print_paths_as_JSON(std::ostream& s, 
    const std::vector<std::unique_ptr<Dubins>>& paths,
    const std::vector<AircraftStats>& stats,
    double min_sep,
    double wind_x, double wind_y, double z_alpha)
{
    assert(paths.size() == stats.size());

    double duration = compute_plan_duration(paths,stats);
    uint N = paths.size();



    s << "{" << std::endl;

    {
        print_field(s,"separation",min_sep,true,1);
        print_field(s,"z_alpha",z_alpha,true,1);
        print_field(s,"wind_x",wind_x,true,1);
        print_field(s,"wind_y",wind_y,true,1);
        print_field(s,"duration",duration,true,1);
        print_field(s,"AC_num",N,true,1);
        
        s << "\t\"trajectories\": [" << std::endl;

        print_trajectory(s,paths[0],stats[0],wind_x,wind_y,duration);

        for(uint i = 1; i < N; i++)
        {
            s << "," << std::endl;
            print_trajectory(s,paths[i],stats[i],wind_x,wind_y,duration);
        }

        s << "\t]" << std::endl;
    }

    s << "}" << std::endl;
}

void DubinsPP::OutputPrinter::print_paths_as_CSV(std::ostream& s, 
    const std::vector<std::unique_ptr<Dubins>>& paths,
    const std::vector<AircraftStats>& stats,
    double wind_x, double wind_y, uint samples)
{
    assert(paths.size() == stats.size());
    assert(samples > 1);

    double duration = compute_plan_duration(paths,stats);

    uint N = paths.size();

    // Write header
    s << "time";
    
    for(uint i = 0; i < N; i++)
    {
        s   << ";X_"     << stats[i].id
            << ";Y_"     << stats[i].id
            << ";Z_"     << stats[i].id
            << ";theta_" << stats[i].id;
    }
    s << std::endl;

    // Compute data
    std::vector<double> times;
    std::vector<std::vector<Pose3D>> pts_storages(N);

    for(uint i = 0; i < samples; i++)
    {
        times.push_back(i*duration/(samples-1));
    }
    
    for(uint i = 0; i < N; i++)
    {
        pts_storages[i] = paths[i]->get_positions(times,stats[i].airspeed,true);
    }

    // Print data
    for(uint t_i = 0; t_i < samples; t_i++)
    {
        // Compute wind deflection
        double dx = wind_x*times[t_i];
        double dy = wind_y*times[t_i];

        s   << times[t_i];

        for(uint i = 0; i < N; i++)
        {
            Pose3D pt = pts_storages[i][t_i];
            
            s   << ";" << dx + pt.x 
                << ";" << dy + pt.y
                << ";" << pt.z
                << ";" << pt.theta;
        }

        s << std::endl;
    }
}