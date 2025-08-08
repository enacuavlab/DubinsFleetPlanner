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

#include "testTemporalDistance.hpp"

[[gnu::const]]
inline Eigen::Vector2d angle_vector(double angle)
{
    return Eigen::Vector2d(std::cos(angle),std::sin(angle));
}

template <DubinsMove m>
inline auto samples_generator(const PathShape<m>& s);

template <>
[[gnu::pure]]
inline auto samples_generator(const PathShape<STRAIGHT>& s)
{
    Eigen::Vector2d p0(s.x,s.y);
    Eigen::Vector2d v(s.p1,s.p2);
    auto f_bind = [=](double t) {return p0 + t*v;};
    return f_bind;
}

template <DubinsMove m>
[[gnu::pure]]
inline auto samples_generator(const PathShape<m>& s)
{
    Eigen::Vector2d p0(s.x,s.y);

    double r        = s.p1;
    double omega    = s.p2;
    double phi0     = s.p4;
    auto f_bind = [=](double t) {return p0 + r*angle_vector(t*omega+phi0);};
    return f_bind;
    
}

template<DubinsMove m1, DubinsMove m2, size_t samples>
std::pair<double,double> sample_temporal_XY_dist(const PathShape<m1>& s1, const PathShape<m2>& s2, double duration)
{
    static_assert(samples > 1);

    auto f1 = samples_generator(s1);
    auto f2 = samples_generator(s2);
    double t = 0.;
    double min_dist = (f1(t) - f2(t)).norm();
    double min_loc = 0.;
    
    for(size_t i = 1; i < samples; i++)
    {
        t = i*duration/(samples-1);
        double dist = (f1(t) - f2(t)).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            min_loc = t;
        }
    }

    return {min_loc,min_dist};
}


TEST(TemporalDistance,Random2DCasesNoDerivatives)
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    for(int i = 0; i < TEST_NUM; i++)
    {
        PathShape<STRAIGHT> s = generate_random_shape<STRAIGHT> (3*i  , TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<LEFT>     l = generate_random_shape<LEFT>     (3*i+1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<RIGHT>    r = generate_random_shape<RIGHT>    (3*i+2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        std::cout << "Random shapes generation n°" << i << " done" << std::endl;

        std::pair<double,double> sampled_sl = sample_temporal_XY_dist<STRAIGHT,LEFT,samples>  (s,l,TEST_PATH_DURATION);
        std::cout << "Sampling SL done" << std::endl;
        std::pair<double,double> sampled_sr = sample_temporal_XY_dist<STRAIGHT,RIGHT,samples> (s,r,TEST_PATH_DURATION);
        std::cout << "Sampling SR done" << std::endl;
        std::pair<double,double> sampled_lr = sample_temporal_XY_dist<LEFT,RIGHT,samples>     (l,r,TEST_PATH_DURATION);
        std::cout << "Sampling LR done" << std::endl;

        std::pair<double,double> test_sl_no_der = temporal_XY_dist<STRAIGHT,LEFT,false>  (s,l,TEST_PATH_DURATION);
        std::pair<double,double> test_sr_no_der = temporal_XY_dist<STRAIGHT,RIGHT,false> (s,r,TEST_PATH_DURATION);
        std::pair<double,double> test_lr_no_der = temporal_XY_dist<LEFT,RIGHT,false>     (l,r,TEST_PATH_DURATION);

        EXPECT_NEAR(sampled_sl.first,test_sl_no_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT loc error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sr.first,test_sr_no_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT loc error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_lr.first,test_lr_no_der.first,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT loc error (sampled vs no derivative)";

        EXPECT_NEAR(sampled_sl.second,test_sl_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT val error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_sr.second,test_sr_no_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT val error (sampled vs no derivative)";
        EXPECT_NEAR(sampled_lr.second,test_lr_no_der.second,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT val error (sampled vs no derivative)";
    }
}

TEST(TemporalDistance,Random2DCasesWithDerivatives)
{
    constexpr size_t samples = static_cast<size_t>(TEST_PATH_DURATION/(DubinsFleetPlanner_PRECISION/10));

    for(int i = 0; i < TEST_NUM; i++)
    {
        PathShape<STRAIGHT> s = generate_random_shape<STRAIGHT> (3*i  , TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<LEFT>     l = generate_random_shape<LEFT>     (3*i+1, TEST_POS_RANGE, TEST_SPEED_RANGE);
        PathShape<RIGHT>    r = generate_random_shape<RIGHT>    (3*i+2, TEST_POS_RANGE, TEST_SPEED_RANGE);
        std::cout << "Random shapes generation n°" << i << " done" << std::endl;

        std::pair<double,double> sampled_sl = sample_temporal_XY_dist<STRAIGHT,LEFT,samples>  (s,l,TEST_PATH_DURATION);
        std::cout << "Sampling SL done" << std::endl;
        std::pair<double,double> sampled_sr = sample_temporal_XY_dist<STRAIGHT,RIGHT,samples> (s,r,TEST_PATH_DURATION);
        std::cout << "Sampling SR done" << std::endl;
        std::pair<double,double> sampled_lr = sample_temporal_XY_dist<LEFT,RIGHT,samples>     (l,r,TEST_PATH_DURATION);
        std::cout << "Sampling LR done" << std::endl;

        std::pair<double,double> test_sl_with_der = temporal_XY_dist<STRAIGHT,LEFT,true>  (s,l,TEST_PATH_DURATION);
        std::pair<double,double> test_sr_with_der = temporal_XY_dist<STRAIGHT,RIGHT,true> (s,r,TEST_PATH_DURATION);
        std::pair<double,double> test_lr_with_der = temporal_XY_dist<LEFT,RIGHT,true>     (l,r,TEST_PATH_DURATION);

        EXPECT_NEAR(sampled_sl.first,test_sl_with_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT loc error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sr.first,test_sr_with_der.first,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT loc error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_lr.first,test_lr_with_der.first,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT loc error (sampled vs with derivative)";

        EXPECT_NEAR(sampled_sl.second,test_sl_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-LEFT val error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_sr.second,test_sr_with_der.second,DubinsFleetPlanner_PRECISION) << "STRAIGHT-RIGHT val error (sampled vs with derivative)";
        EXPECT_NEAR(sampled_lr.second,test_lr_with_der.second,DubinsFleetPlanner_PRECISION) << "LEFT-RIGHT val error (sampled vs with derivative)";
    }
}