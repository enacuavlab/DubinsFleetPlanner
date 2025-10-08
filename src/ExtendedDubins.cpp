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

#include "ExtendedDubins.hpp"


std::vector<std::unique_ptr<Dubins>> generate_all_extended_from_baseDubins(const Pose3D& start, const Pose3D& end, 
    std::vector<double> start_lens, std::vector<double> end_lens,
    double climb, double turn_radius)
{
    std::vector<std::unique_ptr<Dubins>> output,tmp;

    auto unique_ptr_transfer = [&](std::vector<std::unique_ptr<Dubins>>& v)
    {
        for(std::unique_ptr<Dubins>& p : v)
        {
            output.push_back(std::move(p));
        }
    };

    tmp = generate_extended_dubins<STRAIGHT,STRAIGHT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_dubins<STRAIGHT,LEFT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
        
    tmp = generate_extended_dubins<STRAIGHT,RIGHT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);

    

    tmp = generate_extended_dubins<LEFT,STRAIGHT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_dubins<LEFT,LEFT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_dubins<LEFT,RIGHT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);



    tmp = generate_extended_dubins<RIGHT,STRAIGHT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_dubins<RIGHT,LEFT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_dubins<RIGHT,RIGHT,list_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    

    return output;
}



std::vector<std::unique_ptr<Dubins>> generate_all_extended_from_fitted_baseDubins(const Pose3D& start, const Pose3D& end, 
    std::vector<double> start_lens, std::vector<double> end_lens,
    double climb, double turn_radius, double target_len, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output,tmp;

    auto unique_ptr_transfer = [&](std::vector<std::unique_ptr<Dubins>>& v)
    {
        for(std::unique_ptr<Dubins>& p : v)
        {
            output.push_back(std::move(p));
        }
    };

    tmp = generate_extended_fitted_dubins<STRAIGHT,STRAIGHT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);

    tmp = generate_extended_fitted_dubins<STRAIGHT,LEFT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
        
    tmp = generate_extended_fitted_dubins<STRAIGHT,RIGHT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    

    
    tmp = generate_extended_fitted_dubins<LEFT,STRAIGHT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_fitted_dubins<LEFT,LEFT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_fitted_dubins<LEFT,RIGHT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);



    tmp = generate_extended_fitted_dubins<RIGHT,STRAIGHT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_fitted_dubins<RIGHT,LEFT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_extended_fitted_dubins<RIGHT,RIGHT,fit_possible_baseDubins>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    

    return output;
}