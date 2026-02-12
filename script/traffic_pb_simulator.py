#!/usr/bin/python3

import dataclasses
import typing
import subprocess
import pathlib

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.lines import Line2D

Colortype = tuple[float,float,float,float]

import pandas as pd

from traffic.core import Traffic
from traffic.data import samples
from traffic.algorithms import filters


from ProblemGenerator import AC_PP_Problem,ACStats,write_pathplanning_problem_to_CSV
from Dubins import Path,BasicPath,FleetPlan,DubinsMove,Pose2D,Pose3D

from pyproj import Geod,Transformer


from traffic_pb_generator import FlightEndpoints,LatlonPose,extract_flight_endpoints,flight_landing
from ioUtils import print_FleetPlan_to_JSON,parse_trajectories_from_JSON

from plotting import plot_several_pose2d_sequences,transpose_list_of_trajectories
#################### Ongoing flights simulator ####################


def solve_problem(solver:pathlib.Path,src_dir:pathlib.Path,dest_dir:pathlib.Path,
                   separation:float, wind:tuple[float,float],threads:int,obstacle_path:typing.Optional[str]
                   ) -> subprocess.CompletedProcess[bytes]:
    cmd = []
    cmd.append(str(solver.resolve()))
    cmd.append(str(src_dir.resolve()))
    cmd.append(str(dest_dir.resolve()))
    cmd.append(str(separation))
    
    cmd.append(str(wind[0]))
    cmd.append(str(wind[1]))
    
    cmd.append('-t')
    cmd.append(str(threads))
    
    cmd.append('-l')
    
    # cmd.append('-v')
    # cmd.append('2')
    
    if obstacle_path is not None:
        cmd.append('-O')
        cmd.append(str(obstacle_path))
    
    print(cmd)
    output = subprocess.run(cmd)
    print(output)
    output.check_returncode()
    return output

@dataclasses.dataclass
class ArrivalsSimulator:
    solver_path:pathlib.Path
    transformer:Transformer  # Convert (lat,lon) coordinates from WGS84 into some (x,y) coordinates for the solver. Must be a Conformal projection (preserve angles)
    timeshifts:list[pd.Timedelta]
    tasklist:list[FlightEndpoints]
    input_json_path:pathlib.Path    = pathlib.Path("input_json.json")
    output_json_path:pathlib.Path   = pathlib.Path("output_json.json")
    input_csv_path:pathlib.Path     = pathlib.Path("input_csv.csv")
    max_reschedule:int  = 2  # Maximum number of reschedule for each aircraft
    separation:float    = 5. # Overriden by the values in `flying` if it is defined
    z_alpha:float       = 1. # Overriden by the values in `flying` if it is defined
    wind_x:float        = 0. # Overriden by the values in `flying` if it is defined
    wind_y:float        = 0. # Overriden by the values in `flying` if it is defined
    scheduled:typing.Optional[FleetPlan] = None
    schedule_counters:dict[int,int]   = dataclasses.field(init=False) # Dict from AC id to number of reschedulings
    
    __last_schedule:pd.Timestamp  = dataclasses.field(init=False)
    __task_index:dict[int,int]    = dataclasses.field(init=False) # Reverse accessor from Aircraft ID to tasklist index
    __t:pd.Timestamp              = dataclasses.field(init=False)
    __end_of_times:pd.Timestamp   = dataclasses.field(init=False)
    __encountered_exception:typing.Optional[Exception] = dataclasses.field(init=False)
    
    __axes:typing.Optional[Axes] = dataclasses.field(init=False)
    __line_dict:dict[int,Line2D] = dataclasses.field(init=False)
    __traj_dict:dict[int,Line2D] = dataclasses.field(init=False)
    __quiver_dict:dict           = dataclasses.field(init=False)
    
    def attach_axes(self,ax:Axes,set_xylims:bool=True):
        self.__axes = ax
        self.__line_dict = dict()
        self.__traj_dict = dict()
        self.__quiver_dict = dict()
        
        if set_xylims:
            maxx = -np.inf
            maxy = -np.inf
            minx = np.inf
            miny = np.inf
            
            for t in self.tasklist:
                start_p = t.start.to_pose3D(self.transformer,True)
                end_p = t.end.to_pose3D(self.transformer,True)
                
                maxx = max(maxx,start_p.x)
                maxx = max(maxx,end_p.x)
                
                maxy = max(maxy,start_p.y)
                maxy = max(maxy,end_p.y)
                
                minx = min(minx,start_p.x)
                minx = min(minx,end_p.x)
                
                miny = min(miny,start_p.y)
                miny = min(miny,end_p.y)
                
            self.__axes.set_xlim(minx,maxx)
            self.__axes.set_ylim(miny,maxy)
            
            #TODO: Proper labels
            # self.__axes.set_xlabel()
                 
        
    
    def get_task(self,ac_id:int) -> FlightEndpoints:
        return self.tasklist[self.__task_index[ac_id]]
    
    def is_running(self,ac_id:int) -> bool:
        task = self.get_task(ac_id)
        return task.start_time <= self.__t and task.end_time > self.__t
    
    def is_scheduled(self,ac_id:int) -> bool:
        if self.scheduled is None:
            return False
        else:
            return ac_id in self.scheduled._traj_dict.keys()
    
    def __post_init__(self):
        self.schedule_counters = dict()
        self.tasklist.sort(key=lambda fpts : fpts.start_time)
        
        self.__t = self.tasklist[0].start_time
        self.__last_schedule = self.__t
        self.__end_of_times = max(t.end_time for t in self.tasklist)
        self.__task_index = dict()
        self.__encountered_exception = None
        
        self.__axes = None
        
        for i,t in enumerate(self.tasklist):
            self.__task_index[t.stats.id] = i
            self.schedule_counters[t.stats.id] = 0
        
        if self.scheduled is not None:
            self.separation = self.scheduled.separation
            self.z_alpha = self.scheduled.z_alpha
            self.wind_x = self.scheduled.wind_x
            self.wind_y = self.scheduled.wind_y
    
    def step(self,timedelta:pd.Timedelta,reschedule:bool=False,threads:int=0) -> list[tuple[ACStats,Pose3D]]:
        output:list[tuple[ACStats,Pose3D]] = []
        
        ### Time forward
        new_t = timedelta + self.__t
        
        ## Move forward and gather candidates
        
        candidate_acs:set[int] = set()
        added_acs:set[int] = set()
        if self.scheduled is not None:
            self.scheduled = self.scheduled.follow_for(timedelta.total_seconds()/60)
            for s,p in self.scheduled.trajectories:
                candidate_acs.add(s.id)
                output.append((s,p.start))
                added_acs.add(s.id)
                print(f"Adding {s.id} from Pathing ({len(p.sections)} sections, {p.duration()} duration)")
        
              
        for i,t in enumerate(self.tasklist):
            id = t.stats.id
            # If already added via paths, skip
            if id in added_acs:
                continue
            # Task already ended: remove drawing
            if t.end_time <= new_t:
                self.__line_dict[id].set_visible(False)
                self.__quiver_dict[id].set_visible(False)
                continue
            
            # Task yet to begin: skip
            if t.start_time > new_t:
                continue
            
            nt = t.straight_update(new_t - t.start_time)
            candidate_acs.add(id)
            added_acs.add(id)
            output.append((nt.stats,nt.start.to_pose3D(self.transformer,True)))
            print(f"Adding {id} from tasks")
            self.tasklist[i] = nt
            
                
                
        self.__t = new_t
        print(f"Set time to {new_t}")
        
        if self.__axes is not None:
            for s,p in output:
                id = s.id
                try:
                    line = self.__line_dict[id]
                    xs = np.append(line.get_xdata(),p.x)
                    ys = np.append(line.get_ydata(),p.y)
                    line.set_xdata(xs)
                    line.set_ydata(ys)
                except KeyError:
                    xs = [p.x]
                    ys = [p.y]
                    line = self.__axes.plot(xs,ys,alpha=0.5,marker='+',label=id)[0]
                self.__line_dict[id] = line
                color = line.get_color()
                
                
                try:
                    quiver = self.__quiver_dict[id]
                except KeyError:
                    quiver = self.__axes.quiver([p.x],[p.y],[np.cos(p.theta)],[np.sin(p.theta)],angles='xy',pivot='middle',color=color)
                quiver.set_offsets([p.x,p.y])
                quiver.set_UVC([np.cos(p.theta)],[np.sin(p.theta)])
                self.__quiver_dict[id] = quiver
                
        
        # Compute which aircraft can be rescheduled
        if reschedule:
            ## Remove those with no reschedule possible
            no_reschedule_left:set[int] = set()
            for id in candidate_acs:
                if self.schedule_counters[id] >= self.max_reschedule:
                    no_reschedule_left.add(id)
                    
            candidate_acs = candidate_acs-no_reschedule_left
            
            if len(candidate_acs) == 0:
                reschedule = False
        
        # If some aircraft have to be rescheduled, do it
        if reschedule:
            ## Print to JSON the set paths
            obstacles_exist = False
            if self.scheduled is not None:
                
                noncandidate_acs = set(self.scheduled._traj_dict.keys()) - candidate_acs
                
                if len(noncandidate_acs) > 0:
                    print_FleetPlan_to_JSON(self.input_json_path,self.scheduled,noncandidate_acs,True)
                
                    obstacles_exist = True
            
            
            ## Print to CSV the problems to solve
            pp_problems = []
            for id in candidate_acs:
                i = self.__task_index[id]
                task = self.tasklist[i]
                ppp = task.to_AC_PP_Problem(self.transformer,self.timeshifts)
                if self.scheduled is not None and id in self.scheduled._traj_dict.keys():
                    _,p = self.scheduled.get_path(id)
                    ppp.start = p.start
                pp_problems.append(ppp)
                self.schedule_counters[id] += 1
                
            write_pathplanning_problem_to_CSV(self.input_csv_path,pp_problems,True)
            
            ## Call the solver
            try:
                solve_problem(
                    self.solver_path,
                    self.input_csv_path,
                    self.output_json_path,
                    self.separation,
                    (self.wind_x,self.wind_y),
                    threads,
                    str(self.input_json_path.absolute()) if obstacles_exist else None
                )
                
                ## Parse the result and merge
            
                solved = parse_trajectories_from_JSON(self.output_json_path)
                
                if self.scheduled is None:
                    self.scheduled = solved
                else:
                    self.scheduled.merge(solved)
                    
                if self.__axes is not None:
                    list_o_trajs = self.scheduled.sample_poses(50)
                    tranposed = transpose_list_of_trajectories(list_o_trajs)
                    colordict = dict()
                    for k,v in self.__line_dict.items():
                        colordict[k] = v.get_color()
                    
                    _,lines = plot_several_pose2d_sequences(self.__axes,tranposed,colordict,alpha=0.2,linestyle=':')
                    print(len(colordict.keys()),len(lines))
                    for k,l in zip(colordict.keys(),lines):
                        try:
                            self.__traj_dict[k].set_visible(False)
                            del self.__traj_dict[k]
                        except KeyError:
                            pass
                        self.__traj_dict[k] = l
                    
                    
            except Exception as e:
                self.__encountered_exception = e
                print(f"EXCEPTION: {e}")
                
                
        if self.__axes is not None:
            self.__axes.legend()
            self.__axes.set_title(str(new_t))
            plt.pause(5)
            
        return output
    
    def simulate(self, timestep:pd.Timedelta, time_threshold:pd.Timedelta, ac_num_threshold:int,
                 threads:int) -> list[tuple[pd.Timestamp,list[tuple[ACStats,Pose3D]]]]:
        log = []
        
        while self.__t < self.__end_of_times:
            
            do_schedule = False
            if (self.__t - self.__last_schedule) >= time_threshold:
                do_schedule = True
            
            if not(do_schedule):
                unscheduled = 0
                for task in self.tasklist:
                    if task.start_time <= self.__t and task.end_time > self.__t:
                        if self.scheduled is None:
                            unscheduled += 1
                        else:
                            if task.stats.id not in self.scheduled._traj_dict.keys():
                                unscheduled += 1
                
                do_schedule = unscheduled >= ac_num_threshold
            
            
            poss = self.step(timestep,do_schedule,threads)
            log.append((self.__t,poss))
            if self.__encountered_exception is not None:
                break
            
        return log


#################### Entrypoint ####################

def filter_traffic(traffic:Traffic,ICAO_set:typing.Iterable[str]) -> Traffic:
    flight_to_remove = set()
    for flight in traffic:
        if flight_landing(flight,ICAO_set) is None:
            flight_to_remove.add(flight.icao24)
            
    df = traffic.data
    return Traffic(df[~df.icao24.isin(flight_to_remove)]).compute_xy().filter(filters.FilterAboveSigmaMedian()).eval(8) # type: ignore
    

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        'Landing scheduler'
    )
    parser.add_argument('solver',help='Location of the DubinsFleetPlanner solver')
    
    parser.add_argument('-d','--mdist',type=float,help='Minimal distance separating aircraft, in NM. Default to 3.',default=3)
    parser.add_argument('-v','--speed',type=float,help='Nominal speed for aircraft, in knots. Default to 250.',default=250)
    parser.add_argument('-r','--turn-radius',dest='turn_radius',
        type=float, help='Minimal turn radius for all aircraft. Default to 1.33 NM (full turn in 2 minutes at 250 kt)',default=1.33)
    parser.add_argument('-w','--wind',dest='wind', nargs=2,
                        help='XY Wind, as a pair of values (X,Y). Default to (0,0), i.e. no wind.',
                        default=(0,0))
    
    parser.add_argument('--data',help='A Traffic-compatible data file. If not set, use the sample dataset of Traffic',default=None)
    parser.add_argument('--ICAOs',nargs='+',
                        help="List of ICAO codes for airports in which to look for landings in the given dataset. Default to LFPO,LFPG,LFPB",
                        default=["LFPO","LFPG","LFPB"])
    parser.add_argument('-s','--threshold-shift',type=float,dest='threshold_shift',
                        help='Shift distance relative to runway threshold for defining target point, in NM. Default to 20.',default=20)
    parser.add_argument('--epsg',type=int,help="EPSG code for XY projection of (lat,lon) coordinates from WGS84. Default to 9794 (i.e. Lambert-93)",
                        default=9794)
    
    parser.add_argument('-ts','--timestep',type=float,
                        help="Simulation time step, in minutes. Default to 1.",default=1)
    parser.add_argument('-I','--intervals',nargs=3,
                        help="Triplet (min,max,step) defining the offsets to the reference time for the target times. Defined in minutes. Default to (-20,20,2)",
                        default=(-20,20,2))
    parser.add_argument('-m','--max-reschedule',dest="max_reschedule",type=int,
                        help="Maximum number of times a flight can be rescheduled. Default to 3.",default=3)
    parser.add_argument('-n','--ac-threshold',dest='ac_threshold',type=int,
                        help="Minimal number of unscheduled aircraft triggering a rescheduling. Default to 3.", default=3)
    parser.add_argument('-t','--time-threshold',dest='time_threshold',type=float,
                        help="Time threshold (in minutes) triggering rescheduling. Default to 15 minutes.",default=5)
    parser.add_argument('--threads',dest="threads",type=int,
                        help="Number of threads to be used by the solver. 0 allows it to autoselect. Default to 0.",default=0)
    
    args = parser.parse_args()

    solver = args.solver
    print("===== Parsing traffic... =====\n")
    if args.data is None:
        traffic = filter_traffic(samples.quickstart,["LFPO","LFPG","LFPB"])
    else:
        traffic = Traffic.from_file(args.data)
        if traffic is None:
            print("ERROR: Importing traffic failed. Exiting")
            exit(1)
    
    
    endpoints:list[FlightEndpoints] = []
    expected_speed = args.speed # kts
    threshold_shift = args.threshold_shift # NM
    transformer = Transformer.from_crs(
        "EPSG:4326",   # WGS84 (lat, lon)
        f"EPSG:{args.epsg}",
    )
    timeshifts = pd.timedelta_range(f"{args.intervals[0]} minute",f"{args.intervals[1]} minute",freq=f"{float(args.intervals[2])*60}s").to_list()
    
    for i,flight in enumerate(traffic):
        stats = ACStats(
            i,
            expected_speed/60, # Convert from kts (NM/h) to NM/minute
            1.,
            expected_speed/(60*np.pi) # Full circle in 2 minutes
        )
        
        r = extract_flight_endpoints(flight,args.ICAOs,stats,threshold_shift)
        if r is None:
            continue
        else:
            endpoints.append(r[1])
            
    
    print("\n===== Traffic parsing done! =====\nSetting up simulator...")
    
    import matplotlib.pyplot as plt
    plt.ion()
    fig,ax = plt.subplots(figsize=(16/1.5,9/1.5))
    ax.set_aspect('equal')
    fig.tight_layout()
    
    sim = ArrivalsSimulator(pathlib.Path(solver),
                            transformer,
                            timeshifts,
                            endpoints,
                            max_reschedule=args.max_reschedule,
                            separation=args.mdist,
                            wind_x=float(args.wind[0]),
                            wind_y=float(args.wind[1]))
    
    sim.attach_axes(ax)
    
    sim.simulate(pd.Timedelta(f"{args.timestep} minute"),
        pd.Timedelta(f"{args.time_threshold} minute"),
        args.ac_threshold,
        args.threads
    )
    
    
    
    
if __name__ == '__main__':
    main()
