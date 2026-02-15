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

// -------------------- Re-fit Dubins paths with fixed starts and ends -------------------- //

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

// -------------------- Adjust start and end lengths with fixed turn radius -------------------- //

std::vector<std::unique_ptr<Dubins>> generate_line_extended_base(const Pose3D& start, const Pose3D& end, 
    double climb, double turn_radius, double target_len, double tol, const std::vector<double>& ratios)
{
    std::vector<std::unique_ptr<Dubins>> output;

    Pose3D norm_start(start),norm_end(end);
    norm_start.x /= turn_radius;
    norm_start.y /= turn_radius;

    norm_end.x /= turn_radius;
    norm_end.y /= turn_radius;

    for(double r : ratios)
    {
        double LSL_length = turn_radius*fit_shift_LSL_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(LSL_length))
        {
            output.push_back(
                std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLSL,STRAIGHT>>(
                    climb,turn_radius,start,end,LSL_length*r,LSL_length*(1-r)
                )
            );
        }

        double RSR_length = turn_radius*fit_shift_RSR_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(RSR_length))
        {
            output.push_back(
                std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRSR,STRAIGHT>>(
                    climb,turn_radius,start,end,RSR_length*r,RSR_length*(1-r)
                )
            );
        }

        double RSL_length = turn_radius*fit_shift_RSL_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(RSL_length))
        {
            output.push_back(
                std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRSL,STRAIGHT>>(
                    climb,turn_radius,start,end,RSL_length*r,RSL_length*(1-r)
                )
            );
        }

        double LSR_length = turn_radius*fit_shift_LSR_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(LSR_length))
        {
            output.push_back(
                std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLSR,STRAIGHT>>(
                    climb,turn_radius,start,end,LSR_length*r,LSR_length*(1-r)
                )
            );
        }

        double RLR_length = turn_radius*fit_shift_RLR_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(RLR_length))
        {
            output.push_back(
                std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRLR,STRAIGHT>>(
                    climb,turn_radius,start,end,RLR_length*r,RLR_length*(1-r)
                )
            );
        }

        double LRL_length = turn_radius*fit_shift_LRL_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(LRL_length))
        {
            output.push_back(
                std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLRL,STRAIGHT>>(
                    climb,turn_radius,start,end,LRL_length*r,LRL_length*(1-r)
                )
            );
        }

        double SLS_length = turn_radius*fit_shift_SLS_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(RLR_length))
        {
            output.push_back(
                std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsSLS,STRAIGHT>>(
                    climb,turn_radius,start,end,SLS_length*r,SLS_length*(1-r)
                )
            );
        }

        double SRS_length = turn_radius*fit_shift_SRS_ratio(norm_start,norm_end,target_len/turn_radius,tol,r);
        if (!std::isnan(LRL_length))
        {
            output.push_back(
                std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsSRS,STRAIGHT>>(
                    climb,turn_radius,start,end,SRS_length*r,SRS_length*(1-r)
                )
            );
        } 

    }

    return output;
}