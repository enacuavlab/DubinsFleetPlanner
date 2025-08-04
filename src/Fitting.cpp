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
double generic_fit(F f, double min_rho, double max_rho, double tol=1e-6)
{

#if DubinsFleetPlanner_ASSERTIONS > 0
        assert(tol > 0);
#endif

    std::uintmax_t inter_count = 1000;

    auto tol_func = eps_tolerance<double>(-std::log2(tol));

    double fa = f(min_rho);
    double fb = f(max_rho);

    // No root in bracket
    if (boost::math::sign(fa) * boost::math::sign(fb) > 0)
    {
        return NAN;
    }

    std::pair<double,double> bracket = toms748_solve(f,min_rho,max_rho,fa,fb,tol_func,inter_count);

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
    std::pair<double,double> d_bracket = LSL_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,100*min_rho);
    auto target_fun = [=](double rho){return LSL_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_RSR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = RSR_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,100*min_rho);
    auto target_fun = [=](double rho){return RSR_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_RSL(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = RSL_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,100*min_rho);
    auto target_fun = [=](double rho){return RSL_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_LSR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = LSR_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,100*min_rho);
    auto target_fun = [=](double rho){return LSR_total_distance(alpha,beta,d/rho)*rho-target_l;};
    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_RLR(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = RLR_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,100*min_rho);
    auto target_fun = [=](double rho){return RLR_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_LRL(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = LRL_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,100*min_rho);
    auto target_fun = [=](double rho){return LRL_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_SRS(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = SRS_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,100*min_rho);
    auto target_fun = [=](double rho){return SRS_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}

double fit_SLS(double alpha, double beta, double d, double min_rho, double target_l, double tol)
{
    std::pair<double,double> d_bracket = SLS_possible_d(alpha,beta);
    double rmin = std::max(min_rho,d/d_bracket.second);
    double rmax = std::min(d/d_bracket.first,100*min_rho);
    auto target_fun = [=](double rho){return SLS_total_distance(alpha,beta,d/rho)*rho-target_l;};

    return generic_fit(target_fun,rmin,rmax,tol);
}
