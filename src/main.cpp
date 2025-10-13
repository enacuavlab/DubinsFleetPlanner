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

#include <boost/program_options.hpp>

#include <string>
#include <iostream>
#include <unistd.h>

#include <chrono>
#include <ctime>

#include <filesystem>

#include "ioUtils.hpp"
#include "FleetPlanner.hpp"


struct program_arguments
{
    string src_filename;
    string out_filename;
    double separation;
    double wind_x,wind_y;
    vector<double> length_extensions;
    int thread_num;
    int max_iters;
    int samples;
    int verbosity;
    double precision;
    double max_r_length;
};

namespace fs = std::filesystem;
namespace po = boost::program_options;

bool process_command_line(int argc, char* argv[], program_arguments& parsed_args)
{
    // Compute current time, for default file format
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    string datestring(std::ctime(&time_t));
    datestring += ".json";
    
    // Define options
    po::options_description desc("Allowed options");
    desc.add_options()
    ("input"        , po::value<string>(&parsed_args.src_filename)->required()
                    , "REQUIRED, positional. CSV file describing a problem. See 'USAGE.md' for the format description")
    ("output"       , po::value<string>(&parsed_args.out_filename)->required()
                    , "REQUIRED, positional. Output file. Format is deduced by extension type, either JSON or CSV."
                        "Format is described in 'USAGE.md'." )

    ("separation"   , po::value<double>(&parsed_args.separation)->required()
                    ,"REQUIRED, positional. Minimal required XY separation between aircraft")
    
    ("wind-x"       , po::value<double>(&parsed_args.wind_x)->default_value(0.)
                    ,"Positional. Wind speed along the X axis. Default to 0.")
    
    ("wind-y"       , po::value<double>(&parsed_args.wind_y)->default_value(0.)
                    ,"Positional. Wind speed along the Y axis. Default to 0.")
    
    ("samples,s"    , po::value<int>(&parsed_args.samples)->default_value(1000)
                    ,"Number of samples for display and/or exporting to CSV. Default to 1000.")   
    
    ("extended,e"   , po::value<vector<double>>(&parsed_args.length_extensions)->multitoken()
                    ,"Use extended Dubins curves with the given lengths")

    ("threads,t"    , po::value<int>(&parsed_args.thread_num)->default_value(-1)
                    ,"Number of threads to use (if 0, the program chooses automatically). Disabled multi-threading by default.")

    ("precision,p"  , po::value<double>(&parsed_args.precision)->default_value(1e-3)
                    ,"Numeric precision for computations. Default to 1e-3.")
    ("max-r-length,r", po::value<double>(&parsed_args.max_r_length)->default_value(3.)
                    ,"Maximal relative length. Default to 3.")
    ("max-iters,I"  , po::value<int>(&parsed_args.max_iters)->default_value(300)
                    ,"Maximal number of iterations. Default to 300.")

    ("help", "Produce help message")
    ("verbose,v"    , po::value<int>(&parsed_args.verbosity)->default_value(1)
                    , "Set verbosity, from 0 (silent), to 3 (very very verbose). Default to 1.")
    
    ;
    
    // Specify positional arguments
    po::positional_options_description p;
    p.add("input", 1);
    p.add("output", 1);
    p.add("separation", 1);
    p.add("wind-x", 1);
    p.add("wind-y", 1);
    
    // Parse
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
    options(desc).positional(p).run(), vm);
    
    // Check if help was asked (and if it is the case, display then indicate failure)
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return false;
    }
    
    try
    {
        // Check arguments are correct
        po::notify(vm); 
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        
        std::cout << desc << std::endl;
        return false;
    }
    catch(...)
    {
        std::cerr << "Unknown error!" << std::endl;
        return false;
    }
    
    // Declare the supported options.
    return true;
}

int main(int argc, char *argv[])
{
    
    // ----- Parse arguments ----- //
    
    program_arguments args;
    
    bool successful_parse = process_command_line(argc,argv,args);
    
    if (!successful_parse) {exit(0);}
    
    if (args.max_iters <= 0)
    {
        std::cerr   << "Maximal number of iterations is non-positive (Got: " << args.max_iters << " )!" << std::endl 
        << "Exiting now..." << std::endl; 
        exit(0);
    }
    
    if (args.samples < 3)
    {
        std::cerr   << "Number of saples is too small, ie less than 3 (Got: " << args.samples << " )!" << std::endl 
        << "Exiting now..." << std::endl; 
        exit(0);
    }
    
    // ----- Parse input file ----- //
    
    fs::path src_file(args.src_filename);
    std::ifstream src_data(src_file);
    
    if (src_data.bad())
    {
        std::cerr   << "Error while reading trying to open the input file (tried path: " << args.src_filename << " )" << std::endl
        << "Exiting now..." << std::endl;
        exit(0); 
    }
    
    DubinsPP::InputParser::CaseData data = DubinsPP::InputParser::parse_data_csv(src_data);
    
    std::vector<Pose3D> starts,ends;
    std::vector<AircraftStats> stats;
    std::vector<double> dt;
    
    starts  = std::get<0>(data);
    ends    = std::get<1>(data);
    stats   = std::get<2>(data);
    dt      = std::get<3>(data);
    
    // ----- Start optimization ----- //
    
    std::unique_ptr<AbstractFleetPlanner> planner;
    bool good_solution = false;
    
    if (args.length_extensions.size())
    {
        planner = std::make_unique<ExtendedDubinsFleetPlanner>(args.precision,args.max_r_length,args.length_extensions,args.length_extensions); 
    }
    else
    {
        planner = std::make_unique<BasicDubinsFleetPlanner>(args.precision,args.max_r_length);
    }
    
    std::optional<vector<std::shared_ptr<Dubins>>> sols;
    
    if (args.thread_num < 0)
    {
        auto tmp_sols = planner->solve<Dubins::are_XY_separated>(starts,ends,stats,args.separation,
            dt,args.wind_x,args.wind_y,args.max_iters);
            
        if (tmp_sols.has_value())
        {
            sols = make_shared(tmp_sols.value());
            good_solution = true;
        }
        else // If no solution, retry without separation
        {
            sols = std::nullopt;

            std::cerr << "WARNING: Could not find a solution; retrying with SEPARATION DISABLED" << std::endl;

            auto tmp_sols_bis = planner->solve<Dubins::are_XY_separated>(starts,ends,stats,0.,
                dt,args.wind_x,args.wind_y,args.max_iters);

            if (tmp_sols_bis.has_value())
            {
                sols = make_shared(tmp_sols_bis.value());
            }
            else
            {
                sols = std::nullopt;
            }
        }
    }
    else
    {
        sols = planner->solve_parallel<Dubins::are_XY_separated>(starts,ends,stats,args.separation,
            dt,args.wind_x,args.wind_y,args.max_iters,args.thread_num);

        if (!sols.has_value()) // If no solution, retry without separation
        {
            std::cerr << "WARNING: Could not find a solution; retrying with SEPARATION DISABLED" << std::endl;
            
            sols = planner->solve_parallel<Dubins::are_XY_separated>(starts,ends,stats,0.,
                dt,args.wind_x,args.wind_y,args.max_iters,args.thread_num);
        }
        else
        {
            good_solution = true;
        }
    }
        
    if (!sols.has_value())
    {
        std::cerr << "ERROR: Could not find a solution, exiting without writing file..." << std::endl;
        exit(1);
    }
    
    // ----- Print result ----- //
    
    fs::path out_file(args.out_filename);
    std::ofstream out_data(out_file);
    
    string ext;
    if (out_file.has_extension())
    {
        ext = out_file.extension();
    }
    else
    {
        ext = ".json";
    }
    
    // Put to lowercase
    std::transform(ext.begin(), ext.end(), ext.begin(),
    [](unsigned char c){ return std::tolower(c); });
    
    if (ext == ".csv")
    {
        DubinsPP::OutputPrinter::print_paths_as_CSV(out_data,sols.value(),stats,args.wind_x,args.wind_y,args.samples);
    }
    else
    {
        DubinsPP::OutputPrinter::print_paths_as_JSON(out_data,sols.value(),stats,args.separation,args.wind_x,args.wind_y);
    }

    if (good_solution)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

        