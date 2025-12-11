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

std::vector<std::unique_ptr<Dubins>> generate_adjusted_first_straight_base_extended(const Pose3D& start, const Pose3D& end, 
    double climb, double turn_radius, double target_len, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output;

    Pose3D start_norm(start),end_norm(end);
    start_norm.x /= turn_radius;
    start_norm.y /= turn_radius;

    end_norm.x /= turn_radius;
    end_norm.y /= turn_radius;

    
    double LSL_length = turn_radius * fit_shift_LSL_start(start_norm,end_norm,target_len/turn_radius,tol);
    double RSR_length = turn_radius * fit_shift_RSR_start(start_norm,end_norm,target_len/turn_radius,tol);
    double RSL_length = turn_radius * fit_shift_RSL_start(start_norm,end_norm,target_len/turn_radius,tol);
    double LSR_length = turn_radius * fit_shift_LSR_start(start_norm,end_norm,target_len/turn_radius,tol);
    double RLR_length = turn_radius * fit_shift_RLR_start(start_norm,end_norm,target_len/turn_radius,tol);
    double LRL_length = turn_radius * fit_shift_LRL_start(start_norm,end_norm,target_len/turn_radius,tol);

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLSL,STRAIGHT>>(
            climb, turn_radius, start, end, LSL_length, 0., target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRSR,STRAIGHT>>(
            climb, turn_radius, start, end, RSR_length, 0., target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRSL,STRAIGHT>>(
            climb, turn_radius, start, end, RSL_length, 0., target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLSR,STRAIGHT>>(
            climb, turn_radius, start, end, LSR_length, 0., target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRLR,STRAIGHT>>(
            climb, turn_radius, start, end, RLR_length, 0., target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLRL,STRAIGHT>>(
            climb, turn_radius, start, end, LRL_length, 0., target_len, tol
        )
    );

    return output;
}

std::vector<std::unique_ptr<Dubins>> generate_adjusted_last_straight_base_extended(const Pose3D& start, const Pose3D& end, 
    double climb, double turn_radius, double target_len, double tol)
{
    std::vector<std::unique_ptr<Dubins>> output;

    Pose3D start_norm(start),end_norm(end);
    start_norm.x /= turn_radius;
    start_norm.y /= turn_radius;

    end_norm.x /= turn_radius;
    end_norm.y /= turn_radius;

    
    double LSL_length = turn_radius * fit_shift_LSL_end(start_norm,end_norm,target_len/turn_radius,tol);
    double RSR_length = turn_radius * fit_shift_RSR_end(start_norm,end_norm,target_len/turn_radius,tol);
    double RSL_length = turn_radius * fit_shift_RSL_end(start_norm,end_norm,target_len/turn_radius,tol);
    double LSR_length = turn_radius * fit_shift_LSR_end(start_norm,end_norm,target_len/turn_radius,tol);
    double RLR_length = turn_radius * fit_shift_RLR_end(start_norm,end_norm,target_len/turn_radius,tol);
    double LRL_length = turn_radius * fit_shift_LRL_end(start_norm,end_norm,target_len/turn_radius,tol);

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLSL,STRAIGHT>>(
            climb, turn_radius, start, end, 0., LSL_length, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRSR,STRAIGHT>>(
            climb, turn_radius, start, end, 0., RSR_length, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRSL,STRAIGHT>>(
            climb, turn_radius, start, end, 0., RSL_length, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLSR,STRAIGHT>>(
            climb, turn_radius, start, end, 0., LSR_length, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRLR,STRAIGHT>>(
            climb, turn_radius, start, end, 0., RLR_length, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLRL,STRAIGHT>>(
            climb, turn_radius, start, end, 0., LRL_length, target_len, tol
        )
    );

    return output;
}

std::vector<std::unique_ptr<Dubins>> generate_adjusted_both_straight_base_extended(const Pose3D& start, const Pose3D& end, 
    double climb, double turn_radius, double target_len, double tol)
{
     std::vector<std::unique_ptr<Dubins>> output;

    Pose3D start_norm(start),end_norm(end);
    start_norm.x /= turn_radius;
    start_norm.y /= turn_radius;

    end_norm.x /= turn_radius;
    end_norm.y /= turn_radius;

    
    double LSL_length = turn_radius * fit_shift_LSL_both(start_norm,end_norm,target_len/turn_radius,tol);
    double RSR_length = turn_radius * fit_shift_RSR_both(start_norm,end_norm,target_len/turn_radius,tol);
    double RSL_length = turn_radius * fit_shift_RSL_both(start_norm,end_norm,target_len/turn_radius,tol);
    double LSR_length = turn_radius * fit_shift_LSR_both(start_norm,end_norm,target_len/turn_radius,tol);
    double RLR_length = turn_radius * fit_shift_RLR_both(start_norm,end_norm,target_len/turn_radius,tol);
    double LRL_length = turn_radius * fit_shift_LRL_both(start_norm,end_norm,target_len/turn_radius,tol);

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLSL,STRAIGHT>>(
            climb, turn_radius, start, end, LSL_length/2, LSL_length/2, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRSR,STRAIGHT>>(
            climb, turn_radius, start, end, RSR_length/2, RSR_length/2, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRSL,STRAIGHT>>(
            climb, turn_radius, start, end, RSL_length/2, RSL_length/2, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLSR,STRAIGHT>>(
            climb, turn_radius, start, end, LSR_length/2, LSR_length/2, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsRLR,STRAIGHT>>(
            climb, turn_radius, start, end, RLR_length/2, RLR_length/2, target_len, tol
        )
    );

    output.push_back(
        std::make_unique<BaseExtendedDubins<STRAIGHT,BaseDubinsLRL,STRAIGHT>>(
            climb, turn_radius, start, end, LRL_length/2, LRL_length/2, target_len, tol
        )
    );

    return output;
}

std::vector<std::unique_ptr<Dubins>> generate_line_extended_base(const Pose3D& start, const Pose3D& end, 
    double climb, double turn_radius, double target_len, double tol, const std::vector<double>& ratios)
{
    std::vector<std::unique_ptr<Dubins>> output = generate_adjusted_both_straight_base_extended(start,end,
        climb, turn_radius, target_len, tol);

    auto last_straights = generate_adjusted_last_straight_base_extended(
        start,end,climb,turn_radius,target_len,tol
    );

    output.insert(output.end(),
        std::move_iterator(last_straights.begin()),
        std::move_iterator(last_straights.end()));

    auto first_straights = generate_adjusted_first_straight_base_extended(
        start,end,climb,turn_radius,target_len,tol
    );

    output.insert(output.end(),
        std::move_iterator(first_straights.begin()),
        std::move_iterator(first_straights.end()));

    return output;
}