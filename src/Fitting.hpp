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

#include <tuple>
#include <boost/math/tools/roots.hpp>

#include "DubinsPrimitives.hpp"


double fit_LSL(double alpha, double beta, double d, double min_rho);

double fit_RSR(double alpha, double beta, double d, double min_rho);

double fit_RSL(double alpha, double beta, double d, double min_rho);

double fit_LSR(double alpha, double beta, double d, double min_rho);

double fit_RLR(double alpha, double beta, double d, double min_rho);

double fit_LRL(double alpha, double beta, double d, double min_rho);

double fit_SRS(double alpha, double beta, double d, double min_rho);

double fit_SLS(double alpha, double beta, double d, double min_rho);