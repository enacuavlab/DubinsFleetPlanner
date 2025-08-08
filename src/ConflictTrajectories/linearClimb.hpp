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

#include "./circleCircle.hpp"
#include "./lineCircle.hpp"

template<class C>
class LinearClimb : protected C
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    private:
        const double Dz;
        const double vz1,vz2;

    public:
        void print_params()
        {
            C::print_params();
            std::cout   << std::setprecision(5)
                        << "Dz   : " << Dz      << std::endl 
                        << "vz1  : " << vz1     << std::endl
                        << "vz2  : " << vz2     << std::endl;
            
        }

        [[gnu::pure]] double f(double t) const
        {
            double z = Dz + t*(vz1-vz2);
            return C::f(t) + z*z;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            interval<double,Policies> z_I = Dz + I*(vz1-vz2);
            return C::F(I) + square(z_I);
        }

        [[gnu::pure]] double f_d(double t) const
        {
            double z    = Dz + t*(vz1-vz2);
            double z_d  = (vz1-vz2);

            return C::f_d(t) + 2*z*z_d;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_d(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;


            interval<double,Policies> z_I   = Dz + I*(vz1-vz2);
            // interval<double,Policies> z_d_I = (vz1-vz2);
            double z_d = (vz1 - vz2); // Independent from t

            return C::F_d(I) + z_I*z_d*static_cast<double>(2);
        }

        [[gnu::pure]] double f_dd(double t) const
        {
            // double z    = Dz + t*(vz1-vz2);
            double z_d  = (vz1-vz2);
            // double z_dd = 0.;

            return C::f_dd(t) + 2*z_d*z_d;
        } 

        [[gnu::pure]] IntervalSolver::interval<double,Policies> F_dd(const IntervalSolver::interval<double,Policies> &I) const
        {
            using namespace boost::numeric;

            double z_d  = (vz1-vz2);
            // double z_dd = 0.;
            
            return C::F_dd(I) + (2.)*z_d*z_d;
        }
};

template
class LinearClimb<LineCircle>
{
    LinearClimb<LineCircle>(double _Dx, double _Dy, double _r, double _w, double _phi, double _vx, double _vy, double _Dz, double _vz1, double _vz2)
    : LineCircle(_Dx,_Dy,_r,_w,_phi,_vx,_vy), Dz(_Dz),vz1(_vz1),vz2(_vz2) {}

    template<DubinsMove m> requires (m!=STRAIGHT)
    LinearClimb<LineCircle>(const PathShape<STRAIGHT> &s, const PathShape<m> &t)
    : LineCircle(s,t), Dz(s.z - t.z), vz1(s.p3), vz2(t.p3) {}
};

template
class LinearClimb<CircleCircle>
{
    LinearClimb<CircleCircle>(double _Dx, double _Dy, double _r1, double _r2, double _w1, double _w2, double _Dphi, double _Dz, double _vz1, double _vz2)
    : CircleCircle(_Dx,_Dy,_r1,_r2,_w1,_w2,__Dphi), Dz(_Dz),vz1(_vz1),vz2(_vz2) {}

    template<DubinsMove m1, DubinsMove m2> requires ((m1!=STRAIGHT) && (m2!=STRAIGHT))
    LinearClimb<CircleCircle>(const PathShape<m1> &s, const PathShape<m2> &t)
    : CircleCircle(s,t), Dz(s.z - t.z), vz1(s.p3), vz2(t.p3) {}
};