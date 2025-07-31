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

#include "Fitting.hpp"

using namespace boost::math::tools;

template<typename F>
double generic_fit(F f, double min_rho, double target_l, double tol=1e-6)
{

#if DubinsFleetPlanner_ASSERTIONS > 0
        assert(target_l > 0);
        assert(tol > 0);
#endif

    bool increasing;
    double left_val = f(min_rho);
    if (left_val < target_l)
    {
        increasing = true;
    }
    else
    {
        increasing = false;
    }

    std::uintmax_t inter_count = 0;

    auto tol_func = eps_tolerance<double>(-std::log2(tol));

    std::pair<double,double> bracket = bracket_and_solve_root(f,min_rho,2.,increasing,tol_func,inter_count);

    bool valid_bracket  = boost::math::sign(f(bracket.first))*boost::math::sign(f(bracket.second)) <= 0;
    bool precise_enough = tol_func(bracket.first,bracket.second);
    
    if (valid_bracket && precise_enough)
    {
        return (bracket.first+bracket.second)/2;
    }
    else
    {
        return NAN;
    }
}


double fit_LSL(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    auto target_fun = [=](double rho){return LSL_total_distance(alpha,beta,d/rho)*rho;};
    return generic_fit(target_fun,min_rho,target_l,tol);
}

double fit_RSR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    auto target_fun = [=](double rho){return RSR_total_distance(alpha,beta,d/rho)*rho;};
    return generic_fit(target_fun,min_rho,target_l,tol);
}

double fit_RSL(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    auto target_fun = [=](double rho){return RSL_total_distance(alpha,beta,d/rho)*rho;};
    return generic_fit(target_fun,min_rho,target_l,tol);
}

double fit_LSR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    auto target_fun = [=](double rho){return LSR_total_distance(alpha,beta,d/rho)*rho;};
    return generic_fit(target_fun,min_rho,target_l,tol);
}

double fit_RLR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    auto target_fun = [=](double rho){return RLR_total_distance(alpha,beta,d/rho)*rho;};
    return generic_fit(target_fun,min_rho,target_l,tol);
}

double fit_LRL(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    auto target_fun = [=](double rho){return LRL_total_distance(alpha,beta,d/rho)*rho;};
    return generic_fit(target_fun,min_rho,target_l,tol);
}

double fit_SRS(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    auto target_fun = [=](double rho){return SRS_total_distance(alpha,beta,d/rho)*rho;};
    return generic_fit(target_fun,min_rho,target_l,tol);
}

double fit_SLS(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    auto target_fun = [=](double rho){return SLS_total_distance(alpha,beta,d/rho)*rho;};
    return generic_fit(target_fun,min_rho,target_l,tol);
}
