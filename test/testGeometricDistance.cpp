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

#include "testGeometricDistance.hpp"


/**
 * @brief Check if the CSV row matches the expected header row
 * 
 * Reference CSV header: type1,x1,y1,v1_1,v1_2,v1_3,type2,x2,y2,v2_1,v2_2,v2_3,dist
 * 
 * @param r The first CSV row of the file
 * @return true 
 * @return false 
 */
bool check_header(const CSVRow& r)
{
    bool test = (
        (r[0]  == std::string("type1")) &&
        (r[1]  == std::string("x1"))    &&
        (r[2]  == std::string("y1"))    &&
        (r[3]  == std::string("v1_1"))  &&
        (r[4]  == std::string("v1_2"))  &&
        (r[5]  == std::string("v1_3"))  &&
        (r[6]  == std::string("type2")) &&
        (r[7]  == std::string("x2"))    &&
        (r[8]  == std::string("y2"))    &&
        (r[9]  == std::string("v2_1"))  &&
        (r[10] == std::string("v2_2"))  &&
        (r[11] == std::string("v2_3"))  &&
        (r[12] == std::string("dist"))

    );

    return test;
}

template<DubinsMove m>
PathShape<m> parse_from_string(std::string_view x, std::string_view y, std::string_view v1, std::string_view v2, std::string_view v3);

template<>
PathShape<STRAIGHT> parse_from_string(std::string_view x, std::string_view y, std::string_view v1, std::string_view v2, [[maybe_unused]] std::string_view v3)
{
    double _x ;
    std::from_chars(x.data(),x.data()+x.size(),_x);
    double _y ;
    std::from_chars(y.data(),y.data()+y.size(),_y);
    double _v1;
    std::from_chars(v1.data(),v1.data()+v1.size(),_v1);
    double _v2;
    std::from_chars(v2.data(),v2.data()+v2.size(),_v2);

    PathShape<STRAIGHT> output{
        _x,
        _y,
        0.,
        _v1-_x,
        _v2-_y,
        0.,
        0.
    };

    return output;
}

template<DubinsMove m>
PathShape<m> parse_from_string(std::string_view x, std::string_view y, std::string_view v1, std::string_view v2, std::string_view v3)
{
    double _x;
    std::from_chars(x.data(), x.data()+x.size(), _x);
    double _y;
    std::from_chars(y.data(), y.data()+y.size(), _y);
    double _radius;
    std::from_chars(v3.data(), v3.data()+v3.size(), _radius);
    double _phi_low;
    std::from_chars(v1.data(), v1.data()+v1.size(), _phi_low);
    double _phi_high;
    std::from_chars(v2.data(), v2.data()+v2.size(), _phi_high);

    PathShape<m> output{
        _x,
        _y,
        0.,
        _radius,
        1/_radius * (m==LEFT ? 1. : -1),
        0.,
        (m==LEFT ? _phi_low : _phi_high)
    };

    return output;
}

double parse_arc_length(std::string_view v1, std::string_view v2, std::string_view v3)
{
    double _radius;
    std::from_chars(v3.data(), v3.data()+v3.size(), _radius);
    double _phi_low;
    std::from_chars(v1.data(), v1.data()+v1.size(), _phi_low);
    double _phi_high;
    std::from_chars(v2.data(), v2.data()+v2.size(), _phi_high);

    return (_phi_high-_phi_low)*_radius;
}

TEST(GeometricDistance,Random2DCases)
{
    std::ifstream       file(TEST_CSV_FILE);
    bool first = true;
    double duration = 5.;

    for(auto &row: CSVRange(file))
    {
        if (first)
        {
            ASSERT_TRUE(check_header(row));
            first = false;
            continue;
        }

        double distance_gt;
        std::from_chars(row[12].data(), row[12].data()+row[12].size(), distance_gt);

        if ((row[0] == std::string("STRAIGHT")) && (row[6] == std::string("STRAIGHT")))
        {
            PathShape<STRAIGHT> s1 = parse_from_string<STRAIGHT>(row[1],row[2],row[3],row[4],row[5]);
            double s1_length;
            std::from_chars(row[5].data(),row[5].data()+row[5].size(),s1_length);
            path_set_planar_speed(s1,s1_length/duration);

            PathShape<STRAIGHT> s2 = parse_from_string<STRAIGHT>(row[6+1],row[6+2],row[6+3],row[6+4],row[6+5]);
            double s2_length;
            std::from_chars(row[6+5].data(),row[6+5].data()+row[6+5].size(),s2_length);
            path_set_planar_speed(s2,s2_length/duration);

            double distance_computed = geometric_XY_dist<STRAIGHT,STRAIGHT>(s1,s2,duration);
            
            EXPECT_DOUBLE_EQ(distance_gt,distance_computed) << "Error on a STRAIGHT-STRAIGHT case";
        }
        else if ((row[0] == std::string("STRAIGHT")) && (row[6] != std::string("STRAIGHT")))
        {
            PathShape<STRAIGHT> s1 = parse_from_string<STRAIGHT>(row[1],row[2],row[3],row[4],row[5]);
            double s1_length;
            std::from_chars(row[5].data(),row[5].data()+row[5].size(),s1_length);
            path_set_planar_speed(s1,s1_length/duration);

            PathShape<LEFT>     s2      = parse_from_string<LEFT>(row[6+1],row[6+2],row[6+3],row[6+4],row[6+5]);
            PathShape<RIGHT>    s2_bis  = parse_from_string<RIGHT>(row[6+1],row[6+2],row[6+3],row[6+4],row[6+5]);
            double s2_length = parse_arc_length(row[6+3],row[6+4],row[6+5]);
            path_set_planar_speed(s2,s2_length/duration);
            path_set_planar_speed(s2_bis,s2_length/duration);

            double distance_computed        = geometric_XY_dist(s1,s2,duration);
            double distance_computed_bis    = geometric_XY_dist(s1,s2_bis,duration);

            EXPECT_DOUBLE_EQ(distance_gt,distance_computed) << "Error on a STRAIGHT-LEFT case";
            EXPECT_DOUBLE_EQ(distance_gt,distance_computed_bis) << "Error on a STRAIGHT-RIGHT case";
        }
        else if ((row[0] != std::string("STRAIGHT")) && (row[6] == std::string("STRAIGHT")))
        {
            PathShape<LEFT>     s1      = parse_from_string<LEFT>(row[1],row[2],row[3],row[4],row[5]);
            PathShape<RIGHT>    s1_bis  = parse_from_string<RIGHT>(row[1],row[2],row[3],row[4],row[5]);
            double s1_length = parse_arc_length(row[3],row[4],row[5]);
            path_set_planar_speed(s1,s1_length/duration);
            path_set_planar_speed(s1_bis,s1_length/duration);

            PathShape<STRAIGHT> s2      = parse_from_string<STRAIGHT>(row[6+1],row[6+2],row[6+3],row[6+4],row[6+5]);
            double s2_length;
            std::from_chars(row[6+5].data(),row[6+5].data()+row[6+5].size(),s2_length);
            path_set_planar_speed(s2,s2_length/duration);

            double distance_computed        = geometric_XY_dist(s1,s2,duration);
            double distance_computed_bis    = geometric_XY_dist(s1_bis,s2,duration);

            EXPECT_DOUBLE_EQ(distance_gt,distance_computed) << "Error on a LEFT-STRAIGHT case";
            EXPECT_DOUBLE_EQ(distance_gt,distance_computed_bis) << "Error on a RIGHT-STRAIGHT case";
        }
        else //(row[0] != std::string("STRAIGHT")) && (row[6] != std::string("STRAIGHT"))
        {
            PathShape<LEFT>     s1      = parse_from_string<LEFT>(row[1],row[2],row[3],row[4],row[5]);
            PathShape<RIGHT>    s1_bis  = parse_from_string<RIGHT>(row[1],row[2],row[3],row[4],row[5]);
            double s1_length = parse_arc_length(row[3],row[4],row[5]);
            path_set_planar_speed(s1,s1_length/duration);
            path_set_planar_speed(s1_bis,s1_length/duration);

            PathShape<LEFT>     s2      = parse_from_string<LEFT>(row[6+1],row[6+2],row[6+3],row[6+4],row[6+5]);
            PathShape<RIGHT>    s2_bis  = parse_from_string<RIGHT>(row[6+1],row[6+2],row[6+3],row[6+4],row[6+5]);
            double s2_length = parse_arc_length(row[6+3],row[6+4],row[6+5]);
            path_set_planar_speed(s2,s2_length/duration);
            path_set_planar_speed(s2_bis,s2_length/duration);

            double distance_computed_stdstd = geometric_XY_dist(s1,s2,duration);
            double distance_computed_stdbis = geometric_XY_dist(s1_bis,s2_bis,duration);
            double distance_computed_bisstd = geometric_XY_dist(s1_bis,s2,duration);
            double distance_computed_bisbis = geometric_XY_dist(s1_bis,s2_bis,duration);

            EXPECT_DOUBLE_EQ(distance_gt,distance_computed_stdstd) << "Error on a LEFT-LEFT case";
            EXPECT_DOUBLE_EQ(distance_gt,distance_computed_stdbis) << "Error on a LEFT-RIGHT case";
            EXPECT_DOUBLE_EQ(distance_gt,distance_computed_bisstd) << "Error on a RIGHT-LEFT case";
            EXPECT_DOUBLE_EQ(distance_gt,distance_computed_bisbis) << "Error on a RIGHT-RIGHT case";
        }
        
        

    }
}

