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
#include <boost/chrono.hpp>

#include <string>
#include <iostream>
#include <unistd.h>

#include <chrono>
#include <ctime>

#include <filesystem>

#include "ProjectHeader.h"
#include "ioUtils.hpp"
#include "FleetPlanner.hpp"
#include "ConflictList.hpp"

/******************** Util functions ********************/

static void lower_string(std::string& s)
{
    std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c){ return std::tolower(c); });
}

template<Dubins::DubinsSeparationFunction sep_fun>
bool check_solution(const std::vector<std::shared_ptr<Dubins>>& s, const std::vector<AircraftStats>& stats, double min_sep)
{
    SharedListOfPossibilities l;
    
    for(auto &d : s)
    {
        l.push_back(std::vector<std::shared_ptr<Dubins>>({d}));
    }

    std::vector<Conflict_T> conflicts = generic_parallel_compute_separations<sep_fun>(2,l,stats,min_sep);
    // std::vector<Conflict_T> re_conflicts = generic_compute_separations<Dubins::are_XY_separated>(l,stats,min_sep);

    return conflicts.size() == 0;
}

/******************** Argument parsing ********************/

namespace fs = std::filesystem;
namespace po = boost::program_options;
namespace chrono = boost::chrono;


struct program_arguments
{
    string src_pathname;
    string out_pathname;
    double separation;
    double wind_x,wind_y;
    vector<double> length_extensions;
    int thread_num;
    int max_iters,weave_iters;
    double min_weave_dist;
    int samples;
    int verbosity;
    double precision;
    double max_r_length;
    bool legacy;
    bool line_fit;
    int max_solver_time;
};


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
    ("input"        , po::value<string>(&parsed_args.src_pathname)->required()
                    , "REQUIRED, positional. Either a CSV file describing a problem, or a folder containing such files. See 'USAGE.md' for the format description")
    ("output"       , po::value<string>(&parsed_args.out_pathname)->required()
                    , "REQUIRED, positional. Output file/folder (if input is a folder). Format is deduced by extension type, either JSON or CSV."
                        " Format is described in 'USAGE.md'." )

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
    ("weave,w"      , po::value<int>(&parsed_args.weave_iters)->default_value(2)
                    ,"Number of samples to add when resampling between two points. Default to 2.")
    ("weave-dist,wd", po::value<double>(&parsed_args.min_weave_dist)->default_value(0.1)
                    ,"Minimal value required between two time-points for ressampling between them (no ressampling if below). Default to 0.1")

    ("help", "Produce help message")
    ("verbose,v"    , po::value<int>(&parsed_args.verbosity)->default_value(1)
                    , "Set verbosity, from 0 (silent), to 3 (very very verbose). Default to 1.")

    ("legacy"       , po::bool_switch(&parsed_args.legacy)->default_value(false)
                    , "Use a previous implementation (if possible). May decrease performances.")

    ("line,l"       , po::bool_switch(&parsed_args.line_fit)->default_value(false)
                    , "Add paths with minimal turn radius and fitted extra straights at start and end")

    ("max-time,T"   , po::value<int>(&parsed_args.max_solver_time)->default_value(120)
                    , "Maximal allowed runtime for the solver, in seconds. Default to 120s.")
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

    if (vm.count("legacy"))
    {
        parsed_args.legacy = true;
    }
    
    return true;
}

/******************** Problem solving ********************/

void write_result(const fs::path& out_filepath, vector<std::shared_ptr<Dubins>>& sol, vector<AircraftStats>& stats, program_arguments& args)
{

    std::ofstream out_data(out_filepath);
    
    string ext;
    if (out_filepath.has_extension())
    {
        ext = out_filepath.extension();
    }
    else
    {
        ext = ".json";
    }
    
    // Put to lowercase
    lower_string(ext);

    if (args.verbosity > 0)
    {
        std::cout << "Writing result at: " << out_filepath.string() << std::endl << std::endl;
    }
     
    
    if (ext == ".csv")
    {
        DubinsPP::OutputPrinter::print_paths_as_CSV(out_data,sol,stats,args.wind_x,args.wind_y,args.samples);
    }
    else
    {
        DubinsPP::OutputPrinter::print_paths_as_ModernJSON(out_data,sol,stats,args.separation,args.wind_x,args.wind_y);

        // fs::path outbis_filepath = out_filepath.string()+".bis.json";
        // std::cout << "Also printing at:" << std::endl << outbis_filepath.string() << std::endl;
        // std::ofstream outbis_data(outbis_filepath);

        // DubinsPP::OutputPrinter::print_paths_as_ModernJSON(outbis_data,sol,stats,args.separation,args.wind_x,args.wind_y);
    }
}

std::tuple<int,SharedDubinsResults,ExtraPPResults> solve_case(const fs::path& input_path, const fs::path& output_path, program_arguments& args)
{
    ExtraPPResults extra;
    extra.case_name = input_path.string();

    if (args.verbosity > 0)
    {
        std::cout << "Reading file: " << input_path << std::endl;
    }

    std::ifstream data_src(input_path);

    if (data_src.bad())
    {
        std::cerr   << "Error while reading trying to open the input file (tried path: " << input_path << " )" << std::endl
        << "Exiting now..." << std::endl;
        return std::make_tuple(DubinsFleetPlanner_PARSING_FAILURE,std::nullopt,extra);
    }
    
    

    DubinsPP::InputParser::CaseData data = DubinsPP::InputParser::parse_data_csv(data_src);

    std::vector<Pose3D> starts,ends;
    std::vector<AircraftStats> stats;
    std::vector<double> dt;
    
    starts  = std::get<0>(data);
    ends    = std::get<1>(data);
    stats   = std::get<2>(data);
    dt      = std::get<3>(data);
    
    // ----- Start optimization ----- //
    
    std::unique_ptr<AbstractFleetPlanner> planner;
    bool good_solution = true;

    string output_log_pathname = output_path.string() + ".log.json";
    std::ofstream output_log(output_log_pathname);

    output_log << "[";
    
    if (args.length_extensions.size())
    {
        if (args.line_fit)
        {
            planner = std::make_unique<LineExtendedDubinsFleetPlanner>(
                args.precision, args.max_r_length,args.length_extensions,
                args.verbosity, output_log
            );
        }
        else
        {
            if (args.legacy)
            {
                planner = std::make_unique<ExtendedDubinsFleetPlanner>(
                    args.precision, args.max_r_length,
                    args.length_extensions, args.length_extensions,
                    args.verbosity, output_log);
            }
            else
            {
                planner = std::make_unique<BaseExtendedDubinsFleetPlanner>(
                    args.precision, args.max_r_length,
                    args.length_extensions, args.length_extensions,
                    args.verbosity, output_log);
            }
        }
        
    }
    else
    {
        if (args.line_fit)
        {
            std::vector<double> default_ratios = {0.,0.5,1.};

            planner = std::make_unique<LineExtendedDubinsFleetPlanner>(
                args.precision, args.max_r_length,default_ratios,
                args.verbosity, output_log
            );
        }
        else
        {
            planner = std::make_unique<BasicDubinsFleetPlanner>(
                args.precision ,args.max_r_length,
                args.verbosity, output_log);
        }
    }
    
    SharedDubinsResults sols = planner->solve<Dubins::are_XY_separated,Dubins::compute_XY_distance>(extra,starts,ends,stats,args.separation,
            dt,args.wind_x,args.wind_y,args.max_iters,args.weave_iters,args.min_weave_dist,args.thread_num,args.max_solver_time);

    output_log << "]";
    
    if (!sols.has_value()) // If no solution, retry without separation
    {
        ExtraPPResults _backup;
        good_solution = false;
        std::cerr << "WARNING: Could not find a solution; retrying with SEPARATION DISABLED" << std::endl;
        
        sols = planner->solve<Dubins::are_XY_separated,Dubins::compute_XY_distance>(_backup,starts,ends,stats,0.,
            dt,args.wind_x,args.wind_y,args.max_iters,args.weave_iters,args.min_weave_dist,args.thread_num,args.max_solver_time);
    }
        
    if (!sols.has_value())
    {
        std::cerr << "ERROR: Could not find a solution!" << std::endl;
        return std::make_tuple(DubinsFleetPlanner_SOLVING_FAILURE,std::nullopt,extra);
    }
    else
    {
        if (good_solution)
        {
            extra.false_positive = !check_solution<Dubins::are_XY_separated_sampling>(sols.value(),stats,args.separation);
            if (extra.false_positive)
            {
                good_solution = false;
                std::cerr << "ERROR: False positive detected!!!" << std::endl;
            }
        }


        write_result(output_path,sols.value(),stats,args);

        if (good_solution)
        {
            return std::make_tuple(DubinsFleetPlanner_SOLVING_SUCCESS,sols,extra);
        }
        else
        {
            return std::make_tuple(DubinsFleetPlanner_SOLVING_FAILURE,sols,extra);
        }
    }
}



/******************** Main entrypoint ********************/

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
        exit(DubinsFleetPlanner_PARSING_FAILURE);
    }
    
    if (args.samples < 3)
    {
        std::cerr   << "Number of saples is too small, ie less than 3 (Got: " << args.samples << " )!" << std::endl 
        << "Exiting now..." << std::endl; 
        exit(DubinsFleetPlanner_PARSING_FAILURE);
    }
    
    // ----- Parse input file ----- //
    
    fs::path src_path(args.src_pathname);
    fs::path out_path(args.out_pathname);
    

    if (fs::is_directory(src_path))
    {
        // Handle a directory: several test cases

        uint testcase_count = 0;
        uint success_count  = 0;

        vector<fs::path> failures;

        chrono::nanoseconds total_cpu_time(0);
        long unsigned int total_iterations = 0;
        
        if (!fs::is_directory(out_path))
        {
            if (!fs::create_directory(out_path))
            {
                std::cerr << "ERROR: Failure when creating ouput directory: " << out_path << std::endl
                << "Exiting now..." << std::endl;

                exit(DubinsFleetPlanner_PARSING_FAILURE);
            }
        }

        fs::path summary_file_path = out_path / "summary.csv";
        std::ofstream summary_file(summary_file_path);

        if (summary_file.bad())
        {
            std::cerr << "ERROR: Could not create summary output file at: " << summary_file_path << std::endl
            << "Exiting now..." << std::endl;

            exit(DubinsFleetPlanner_PARSING_FAILURE);
        }

        summary_file << ExtraPPResults::CSV_header() << std::endl;
        
        for(auto srcfile : fs::directory_iterator(src_path))
        {
            if (fs::is_regular_file(srcfile))
            {
                fs::path out_filepath(out_path);
                string ext = "sol";
                ext.append((out_path.has_extension()) ? (out_path.extension()) : (".json"));

                out_filepath /= srcfile.path().filename();
                out_filepath.replace_extension(ext);

                auto __ret = solve_case(srcfile,out_filepath,args);

                int code = std::get<0>(__ret);
                ExtraPPResults& extra = std::get<2>(__ret);

                total_cpu_time      += extra.duration;
                total_iterations    += extra.iterations;

                if (code == DubinsFleetPlanner_SOLVING_SUCCESS)
                {
                    success_count++;
                }
                else
                {
                    failures.push_back(srcfile.path());
                }

                testcase_count++;

                summary_file << extra.as_CSV() << std::endl;
            }
        }

        summary_file.close();

        
        std::cout   << "List of failures:" << std::endl;
        
        for(auto& inpath : failures)
        {
            std::cout << inpath << std::endl;
        }

        std::cout   << std::endl
                    << "Succes ratio: " << success_count << " / " << testcase_count << std::endl
                    << "Succes rate : " << 100*(static_cast<double>(success_count)/static_cast<double>(testcase_count)) << " %" << std::endl 
                    << "Average time per case (s)       : " << chrono::duration<double>(total_cpu_time)/(testcase_count) << std::endl
                    << "Average time per iteration (s)  : " << chrono::duration<double>(total_cpu_time)/(total_iterations) << std::endl
                    << std::endl;
    }
    else
    {
        // Handle a single file
        auto __ret = solve_case(src_path,out_path,args);
        
        int code = std::get<0>(__ret);
        ExtraPPResults& extra = std::get<2>(__ret);

        std::cout << extra.format() << std::endl;

        exit(code);
    }
}

        