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

#include "BaseExtendedDubins.hpp"


std::vector<std::unique_ptr<Dubins>> generate_base_extended_dubins(const Pose3D& start, const Pose3D& end, 
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

    tmp = generate_base_extended_dubins<STRAIGHT,STRAIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);

    tmp = generate_base_extended_dubins<STRAIGHT,LEFT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
        
    tmp = generate_base_extended_dubins<STRAIGHT,RIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);

    

    tmp = generate_base_extended_dubins<LEFT,STRAIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    
    tmp = generate_base_extended_dubins<LEFT,LEFT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    
    tmp = generate_base_extended_dubins<LEFT,RIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);



    tmp = generate_base_extended_dubins<RIGHT,STRAIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    
    tmp = generate_base_extended_dubins<RIGHT,LEFT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    
    tmp = generate_base_extended_dubins<RIGHT,RIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius);

    unique_ptr_transfer(tmp);
    

    return output;
}



std::vector<std::unique_ptr<Dubins>> generate_all_fitted_base_extended(const Pose3D& start, const Pose3D& end, 
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

    tmp = generate_base_extended_fitted_dubins<STRAIGHT,STRAIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);

    tmp = generate_base_extended_fitted_dubins<STRAIGHT,LEFT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
        
    tmp = generate_base_extended_fitted_dubins<STRAIGHT,RIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    

    
    tmp = generate_base_extended_fitted_dubins<LEFT,STRAIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_base_extended_fitted_dubins<LEFT,LEFT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_base_extended_fitted_dubins<LEFT,RIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);



    tmp = generate_base_extended_fitted_dubins<RIGHT,STRAIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_base_extended_fitted_dubins<RIGHT,LEFT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    
    tmp = generate_base_extended_fitted_dubins<RIGHT,RIGHT>(
        start,end,start_lens,end_lens,climb,turn_radius,target_len,tol);

    unique_ptr_transfer(tmp);
    

    return output;
}