// Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
// 
// This file is part of PH_Spline.
// 
// PH_Spline is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// PH_Spline is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with PH_Spline.  If not, see <https://www.gnu.org/licenses/>.

#include "Dubins.hpp"

/********************  Class implementations  ********************/

/********** LSL Lengths **********/

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::normalized_length(double a, double b, double d)
{
    return LSL_total_distance(a,b,d);
}


/********** RSR Lengths **********/

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::normalized_length(double a, double b, double d)
{
    return RSR_total_distance(a,b,d);
}


/********** RSL Lengths **********/

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::STRAIGHT,DubinsMove::LEFT>::normalized_length(double a, double b, double d)
{
    return RSL_total_distance(a,b,d);
}


/********** LSR Lengths **********/

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::STRAIGHT,DubinsMove::RIGHT>::normalized_length(double a, double b, double d)
{
    return LSR_total_distance(a,b,d);
}


/********** RLR Lengths **********/

template<>
double BaseDubins<DubinsMove::RIGHT,DubinsMove::LEFT,DubinsMove::RIGHT>::normalized_length(double a, double b, double d)
{
    return RLR_total_distance(a,b,d);
}


/********** LRL Lengths **********/

template<>
double BaseDubins<DubinsMove::LEFT,DubinsMove::RIGHT,DubinsMove::LEFT>::normalized_length(double a, double b, double d)
{
    return LRL_total_distance(a,b,d);
}


/********** SRS Lengths **********/

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::RIGHT,DubinsMove::STRAIGHT>::normalized_length(double a, double b, double d)
{
    return SRS_total_distance(a,b,d);
}


/********** SLS Lengths **********/

template<>
double BaseDubins<DubinsMove::STRAIGHT,DubinsMove::LEFT,DubinsMove::STRAIGHT>::normalized_length(double a, double b, double d)
{
    return SLS_total_distance(a,b,d);
}


/********** Other **********/
// We always return NAN as we don't want acceptable paths for the other cases

template<DubinsMove fst, DubinsMove snd, DubinsMove trd>
double BaseDubins<fst,snd,trd>::normalized_length(double a, double b , double d)
{
    return NAN;
}
