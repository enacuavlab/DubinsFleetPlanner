# Copyright (C 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of DubinsFleetPlanner.
# 
# DubinsFleetPlanner is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DubinsFleetPlanner is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

import json,csv

from Dubins import Pose3D,ACStats,Path,BasicPath,FleetPlan,ListOfTimedPoses,TimedPosesLine,DictOfPoseTrajectories

######################################## JSON parsing ########################################

def parse_section_from_dict(d:dict) -> BasicPath:
    return BasicPath(**d)

def parse_trajectory_from_dict(d:dict) -> tuple[ACStats,Path]:
    stats_dict  = d["stats"]
    stats = ACStats(**stats_dict)
    
    path_dict   = d["path"]
    sections = [parse_section_from_dict(t) for t in path_dict["sections"]]
    
    path = Path(
        path_dict["total_length"],
        Pose3D(**path_dict["start"]),
        Pose3D(**path_dict["end"]),
        sections
    )
    
    return stats,path

def parse_trajectories_from_JSON(file) -> FleetPlan:
    with open(file) as jsonfile:
        raw_data = json.load(jsonfile)
        
        paths = [parse_trajectory_from_dict(t) for t in raw_data["trajectories"]]
        output = FleetPlan(
            raw_data["separation"],
            raw_data["z_alpha"],
            raw_data["wind_x"],
            raw_data["wind_y"],
            raw_data["duration"],
            raw_data["AC_num"],
            paths
        )
        
    return output
    
def print_FleetPlan_to_JSON(file,plan:FleetPlan):
    with open(file,mode='x') as jsonfile:
        json.dump(plan.asdict(),jsonfile)


def join_trajectories_JSONs(output_file,*input_files):
    base_plan = parse_trajectories_from_JSON(input_files[0])
    
    for f in input_files[1:]:
        other_plan = parse_trajectories_from_JSON(f)
        base_plan.join(other_plan)
        
    print_FleetPlan_to_JSON(output_file,base_plan)

######################################## CSV Parsing ########################################

def parse_trajectories_from_CSV(file) -> ListOfTimedPoses:
    with open(file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=';', quotechar='|')
        header = next(reader)
        ids = []
        
        # Check header
        assert(header[0].lower() == "time")
        
        for i in range(1,len(header),4):
            assert(header[i].lower()[0] == "x" and\
                header[i+1].lower()[0] == "y" and\
                header[i+2].lower()[0] == "z" and\
                header[i+3].lower()[:5] == "theta")
            ids.append(int(header[i][2:]))
        
        output:ListOfTimedPoses = list()
        for row in reader:
            ts = float(row[0])
            l = []
            for i in range(1,len(row),4):
                id = ids[(i-1)//4]
                x = float(row[i])
                y = float(row[i+1])
                z = float(row[i+2])
                theta = float(row[i+3])
                l.append((id,Pose3D(x,y,z,theta)))
                
            output.append((ts,l))
            
            
        return output
    
def transpose_list_of_trajectories(l:ListOfTimedPoses) -> DictOfPoseTrajectories:
    output:DictOfPoseTrajectories = dict()
    for r in l:
        ts = r[0]
        for ac_id,pose in r[1]:
            try:
                output[ac_id].append((ts,pose))
            except KeyError:
                output[ac_id] = [(ts,pose)]
    
    return output

def make_CSV_trajectories_header(dataline:TimedPosesLine) -> list[str]:
    ac_count = len(dataline[1])
    output = ["time"]
    for i in range(ac_count):
        id = dataline[1][i][0]
        output.extend([f"X_{id}",f"Y_{id}",f"Z_{id}",f"theta_{id}"])
    return output

def print_trajectories_to_CSV(output_file,data:ListOfTimedPoses,time_offset:float=0.):
    with open(output_file, newline='', mode='x') as outcsv:
        writer = csv.writer(outcsv,delimiter=';')
        
        # Write header
        header = make_CSV_trajectories_header(data[0])
        writer.writerow(header)
        
        for line in data:
            row = [line[0]+time_offset]
            for t in line[1]:
                row.extend([t[1].x,t[1].y,t[1].z,t[1].theta])
            writer.writerow(row)


def join_trajectories_CSVs(output_file,*input_files):
    
    last_time = 0.
    previous_header = None
    with open(output_file, newline='', mode='x') as outcsv:
        writer = csv.writer(outcsv,delimiter=';')
        for file in input_files:
            data = parse_trajectories_from_CSV(file)
            if previous_header is None:
                previous_header = make_CSV_trajectories_header(data[0])
                writer.writerow(previous_header)
            else:
                assert previous_header == make_CSV_trajectories_header(data[0])
                
            for line in data:
                row = [line[0]+last_time]
                for t in line[1]:
                    row.extend([t[1].x,t[1].y,t[1].z,t[1].theta])
                writer.writerow(row)
                
            last_time += data[-1][0]
          
          
######################################## Program entry point ########################################       
            
if __name__ == '__main__':
    import argparse
    import pathlib
    
    parser = argparse.ArgumentParser('Data manip','Combine and modify Dubins Fleet Path Plan files.')
    parser.add_argument('files',type=str,nargs='+',help='File(s) from which to parse data')
    parser.add_argument('-j','--join',type=str,help='Join several files into one by putting trajectories end-to-end',default=None)
    
    
    args = parser.parse_args()
    
    paths = [pathlib.Path(f) for f in args.files]
    ext = paths[0].suffix.lower()

    for p in paths:
        try:
            assert(p.is_file())
        except AssertionError:
            raise ValueError(f"The following path does not indicate a file:\n{p}")
        
        try:
            assert(p.suffix.lower() == ext)
        except AssertionError:
            raise ValueError(f"The following file does not have the same extension as the others (got: '{p.suffix.lower()}', expected: '{ext}')\n{p}")
        
    
    if ext == ".csv" and args.join is not None:
        join_trajectories_CSVs(args.join,*paths)
        exit(0)
    elif ext == ".json" and args.join is not None:
        join_trajectories_JSONs(args.join,*paths)
        exit(0)
    
        
        
        
    print("No action selected... Do nothing and exit...")
        
                
            
            
            
        