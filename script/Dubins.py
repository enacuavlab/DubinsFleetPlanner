# Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
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

from __future__ import annotations
import typing

from dataclasses import dataclass,field,asdict
from enum import IntEnum

import numpy as np

#################### Base elements ####################

class DubinsMove(IntEnum):
    STRAIGHT = 0
    LEFT     = 1
    RIGHT    = 2
    
    def __str__(self) -> str:
        return self.name
    
    def abbr(self) -> str:
        return self.name[0]

class Pose2D(typing.NamedTuple):
    x:float
    y:float
    angle:float

@dataclass
class Pose3D:
    x:float # X coordinate
    y:float # Y coordinate
    z:float # Z coordinate
    theta:float # XY orientation, in radian
    
    def to_numpy(self) -> np.ndarray:
        return np.array([self.x,self.y,self.z,self.theta])
    
    @staticmethod
    def from_array(array:typing.Sequence[float]) -> Pose3D:
        return Pose3D(array[0],array[1],array[2],array[3])
    
    def asdict(self) -> dict[str,float]:
        return asdict(self)
    
    def astuple(self) -> tuple[float,float,float,float]:
        return (self.x,self.y,self.z,self.theta)
    
    @staticmethod
    def undefined() -> Pose3D:
        return Pose3D(np.nan,np.nan,np.nan,np.nan)
    
    def to2D(self) -> Pose2D:
        return Pose2D(self.x,self.y,self.theta)
        
def poses_dist(p1:Pose3D,p2:Pose3D) -> float:
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    
    dtheta = p1.theta - p2.theta
    dvx = np.cos(dtheta) - 1.
    dvy = np.sin(dtheta) - 0
    
    return np.sqrt(dx*dx + dy*dy + dz*dz + dvx*dvx + dvy*dvy)

def poses_XY_dist(p1:Pose3D,p2:Pose3D) -> float:
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return np.sqrt(dx*dx+dy*dy)

def min_XY_dist(poses:list[Pose3D]) -> tuple[float,int,int]:
    mdist = np.inf
    n = len(poses)
    i1 = 0
    i2 = 0
    
    for i in range(n):
        for j in range(i+1,n):
            dist = poses_XY_dist(poses[i],poses[j])
            if dist < mdist:
                mdist = dist
                i1 = i
                i2 = j
                
    return mdist,i1,i2

TimedPosesLine = tuple[float,dict[int,Pose3D]]
ListOfTimedPoses = list[TimedPosesLine]
DictOfPoseTrajectories = dict[int,list[tuple[float,Pose3D]]]



################ Base Dubins generation ################

def mod2pi(theta:float) -> float:
    return np.mod(theta, 2.0 * np.pi)

def central_angle(theta:float) -> float:
    output = np.fmod(theta, 2*np.pi)
    
    if output > np.pi:
        return output - 2*np.pi
    
    if output < -np.pi:
        return output + 2*np.pi
    
    return output

def transform(start:Pose2D, goal:Pose2D) -> tuple[float,float,float]:
    x0, y0, th0 = start
    x1, y1, th1 = goal

    dx = x1 - x0
    dy = y1 - y0

    phi = np.arctan2(dy, dx)
    d = np.hypot(dx, dy)

    alpha = mod2pi(th0 - phi)
    beta  = mod2pi(th1 - phi)

    return d, alpha, beta


def LSL(start:Pose2D, goal:Pose2D) -> typing.Optional[tuple[float,float,float]]:
    d, a, b = transform(start, goal)

    sa, sb = np.sin(a), np.sin(b)
    ca, cb = np.cos(a), np.cos(b)

    tmp = d**2 + 2 - 2*np.cos(a - b) + 2*d*(sa - sb)
    if tmp < 0:
        return None

    u = np.sqrt(tmp)
    theta = np.arctan2(cb - ca, d + sa - sb)

    t = mod2pi(theta - a)
    v = mod2pi(b - theta)

    return t, u, v

def RSR(start:Pose2D, goal:Pose2D) -> typing.Optional[tuple[float,float,float]]:
    d, a, b = transform(start, goal)

    sa, sb = np.sin(a), np.sin(b)
    ca, cb = np.cos(a), np.cos(b)

    tmp = d**2 + 2 - 2*np.cos(a - b) + 2*d*(sb - sa)
    if tmp < 0:
        return None

    u = np.sqrt(tmp)
    theta = np.arctan2(ca - cb, d - sa + sb)

    t = mod2pi(a - theta)
    v = mod2pi(theta - b)

    return t, u, v

def LSR(start:Pose2D, goal:Pose2D) -> typing.Optional[tuple[float,float,float]]:
    d, a, b = transform(start, goal)

    sa, sb = np.sin(a), np.sin(b)
    ca, cb = np.cos(a), np.cos(b)

    tmp = d**2 - 2 + 2*np.cos(a - b) + 2*d*(sa + sb)
    if tmp < 0:
        return None

    u = np.sqrt(tmp)
    theta = np.arctan2(-ca - cb, d + sa + sb)

    t = mod2pi(theta - a - np.atan2(-2,u))
    v = mod2pi(theta - b - np.atan2(-2,u))

    return t, u, v

def RSL(start:Pose2D, goal:Pose2D) -> typing.Optional[tuple[float,float,float]]:
    d, a, b = transform(start, goal)

    sa, sb = np.sin(a), np.sin(b)
    ca, cb = np.cos(a), np.cos(b)

    tmp = d**2 - 2 + 2*np.cos(a - b) - 2*d*(sa + sb)
    if tmp < 0:
        return None

    u = np.sqrt(tmp)
    theta = np.arctan2(ca + cb, d - sa - sb)

    t = mod2pi(a - theta + np.atan2(2,u))
    v = mod2pi(b - theta + np.atan2(2,u))

    return t, u, v

def RLR(start:Pose2D, goal:Pose2D) -> typing.Optional[tuple[float,float,float]]:
    d, a, b = transform(start, goal)

    sa, sb = np.sin(a), np.sin(b)
    ca, cb = np.cos(a), np.cos(b)

    tmp = (6 - d**2 + 2*np.cos(a - b) + 2*d*(sa - sb)) / 8
    if np.abs(tmp) > 1:
        return None

    u = mod2pi(2*np.pi - np.arccos(tmp))
    theta = np.arctan2(ca - cb, d - sa + sb)

    t = mod2pi(a - theta + u / 2)
    v = mod2pi(a - b - t + u)

    return t, u, v

def LRL(start:Pose2D, goal:Pose2D) -> typing.Optional[tuple[float,float,float]]:
    d, a, b = transform(start, goal)

    sa, sb = np.sin(a), np.sin(b)
    ca, cb = np.cos(a), np.cos(b)

    tmp = (6 - d**2 + 2*np.cos(a - b) + 2*d*(sb - sa)) / 8
    if np.abs(tmp) > 1:
        return None

    u = mod2pi(2*np.pi - np.arccos(tmp))
    theta = np.arctan2(ca - cb, d + sa - sb)

    t = mod2pi(-a - theta + u / 2)
    v = mod2pi(b - a - t + u)

    return t, u, v

def SRS(start:Pose2D, goal:Pose2D) -> typing.Optional[tuple[float,float,float]]:
    d, a, b = transform(start, goal)
    
    da = central_angle(a - b)
    if np.isclose(da,0.0):
        return None
    
    cs = d * abs(np.sin(b)/np.sin(da))
    ec = d * abs(np.sin(a)/np.sin(da))
    
    
    first_l = cs - 1/np.tan((np.pi-da)/2)
    second_l = mod2pi(a - b)
    third_l = ec - 1/np.tan((np.pi-da)/2)
    
    if first_l < 0 or third_l < 0:
        return None
    else:
        return first_l,second_l,third_l
    
def SLS(start:Pose2D, goal:Pose2D) -> typing.Optional[tuple[float,float,float]]:
    d, a, b = transform(start, goal)
    
    da = central_angle(b - a)
    if np.isclose(da,0.0):
        return None
    
    cs = d * np.abs(np.sin(b)/np.sin(da))
    ec = d * np.abs(np.sin(a)/np.sin(da))
    
    first_l = cs - 1/np.tan((np.pi-da)/2)
    second_l = mod2pi(b - a)
    third_l = ec - 1/np.tan((np.pi-da)/2)
    
    if first_l < 0 or third_l < 0:
        return None
    else:
        return first_l,second_l,third_l
    
    
def all_paths(start:Pose2D, goal:Pose2D) -> tuple[\
                                typing.Optional[tuple[float,float,float]],\
                                typing.Optional[tuple[float,float,float]],\
                                typing.Optional[tuple[float,float,float]],\
                                typing.Optional[tuple[float,float,float]],\
                                typing.Optional[tuple[float,float,float]],\
                                typing.Optional[tuple[float,float,float]],\
                                typing.Optional[tuple[float,float,float]],\
                                typing.Optional[tuple[float,float,float]]]:
    """Returns paths component lengths (or None is solution does not exists) for
    LSL, RSR, LSR, RSL, RLR, LRL, SRS, SLS in that order
    """

    return (LSL(start,goal),
            RSR(start,goal),
            LSR(start,goal),
            RSL(start,goal),
            RLR(start,goal),
            LRL(start,goal),
            SRS(start,goal),
            SLS(start,goal),)

#################### Paths elements ####################

@dataclass
class BasicPath:
    type  : DubinsMove # Either LEFT,RIGHT or STRAIGHT, for describing a turn or a straight line movement,
    length: float # Length of this section
    x     : float # X coordinate of the refence point; For a Straight: Starting point | For a turn: Circle center
    y     : float # Y coordinate of the refence point
    z     : float # Z coordinate of the refence point
    p1    : float # For a Straight: horizontal x speed | For a turn: Radius
    p2    : float # For a Straight: horizontal y speed | For a turn: signed angular speed; positive when LEFT, negative when RIGHT
    p3    : float # Vertical speed
    p4    : float # For a Straight: unused | For a turn: initial angle
    
    def __post_init__(self):
        if type(self.type) is str:
            if self.type == "STRAIGHT":
                self.type = DubinsMove.STRAIGHT
            elif self.type == "LEFT":
                self.type = DubinsMove.LEFT
            elif self.type == "RIGHT":
                self.type = DubinsMove.RIGHT
            else:
                raise ValueError(f"Unknown Dubins Move:\n{self.type}")
        elif type(self.type) is not DubinsMove:
            self.type = DubinsMove(self.type)
    
    def speed(self) -> float:
        if self.type == DubinsMove.STRAIGHT:
            output = np.sqrt(self.p1*self.p1 + self.p2*self.p2)
        else:
            output = abs(self.p1*self.p2)
        return output
        
    def pose_at(self,t:float) -> Pose3D:
        z = self.z + t*self.p3
        
        if self.type == DubinsMove.STRAIGHT:
            x = self.x + t*self.p1
            y = self.y + t*self.p2
            angle = np.arctan2(self.p2,self.p1)
            return Pose3D(x,y,z,angle)
        else:
            center = np.array([self.x,self.y])
            a = self.p4 + t*self.p2
            
            xy_pos = center + self.p1*np.array([np.cos(a),np.sin(a)])
            
            angle = a + (np.pi/2 if self.type == DubinsMove.LEFT else -np.pi/2)
            return Pose3D(xy_pos[0],xy_pos[1],z,angle)
        
    def start(self) -> Pose3D:
        return self.pose_at(0)
    
    def end(self) -> Pose3D:
        return self.pose_at(self.length/self.speed())
    
    def duration(self) -> float:
        if self.length == 0.:
            return 0.
        else:
            return self.length/self.speed()
    
    def asdict(self) -> dict[str,float|DubinsMove]:
        output = asdict(self)
        output["type"] = str(output["type"])
        return output
    
    @staticmethod
    def from_2D(type: DubinsMove, length: float, start:Pose2D, radius:float, speed:float=1.) -> BasicPath:
        dx = np.cos(start.angle)
        vx = dx*speed
        
        dy = np.sin(start.angle)
        vy = dy*speed
        
        if type is DubinsMove.STRAIGHT:
            return BasicPath(type,length,
                             start.x,start.y,0.,
                             vx,vy,
                             0.,0.)
            
        else:
            z = 0.
            p1 = radius
            p2 = speed/radius
            p3 = 0.
            
            if type is DubinsMove.LEFT:
                cx = start.x - dy*radius
                cy = start.y + dx*radius
            else:
                cx = start.x + dy*radius
                cy = start.y - dx*radius
                p2 *= -1
            
            p4 = np.atan2(start.y - cy, start.x - cx)
            return BasicPath(type,length,
                             cx,cy,0.,
                             p1,p2,p3,p4)
    
    
@dataclass
class Path:
    total_length:float  # Total length of the path
    start       :Pose3D # Starting pose
    end         :Pose3D # Ending pose
    sections    :list[BasicPath]    # List of base primitives describing the shape
    junctions   :list[float] = field(init=False) # List of times at which there is a section change
    
    @staticmethod
    def __compute_junctions(sections:list[BasicPath]) -> list[float]:
        output = []
        last_time = 0.
        for s in sections[:-1]:
            last_time += s.duration()
            output.append(last_time)
        return output
    
    def __post_init__(self):
        self.junctions = self.__compute_junctions(self.sections)
    
    def duration(self) -> float:
        if len(self.junctions) > 0:
            return self.junctions[-1] + self.sections[-1].duration()
        else:
            return self.sections[-1].duration()
        
    def pose_at(self,t:float) -> Pose3D:
        section_id = 0
        time = t
        if time > self.duration():
            return Pose3D.undefined()
        
        while section_id < len(self.junctions) and t > self.junctions[section_id]:
            time -= self.sections[section_id].duration()
            section_id += 1
        
        if (section_id >= len(self.junctions)):
            return self.sections[-1].pose_at(time)
        else:
            return self.sections[section_id].pose_at(time)
        
    
    def asdict(self) -> dict[str,typing.Any]:
        output = asdict(self)
        output.pop("junctions")
        return output
    
    def join(self,other:typing.Self):
        self.total_length += other.total_length
        self.end = other.end
        
        s_duration = self.duration()
        self.junctions.append(s_duration)
        
        o_junctions = other.junctions.copy()
        for i in range(len(o_junctions)):
            o_junctions[i] += s_duration
        self.junctions.extend(o_junctions)
        
        self.sections.extend(other.sections)
        
        
    def abbr(self) -> str:
        return ''.join(s.type.abbr() for s in self.sections)
                

@dataclass
class ACStats:
    id          :int
    airspeed    :float # Speed in air referential
    climb       :float # Vertical change in elevation (both directions)
    turn_radius :float # Minimal turn radius
    
    def asdict(self) -> dict[str,int|float]:
        return asdict(self)
    

@dataclass
class FleetPlan:
    separation  :float # Minimal separation achieved
    z_alpha     :float # Z distance modifier value
    wind_x      :float # Wind speed along the X axis 
    wind_y      :float # Wind speed along the Y axis
    duration    :float # Time duration of the plan
    AC_num      :int   # Number of aircraft 
    trajectories:list[tuple[ACStats,Path]] # List of paths and stats
    _traj_dict  :dict[int,int] = field(init=False) # Match each AC_id to the correct index in the `trajectories` list
    
    def __post_init__(self):
        self._traj_dict = dict()
        for i,t in enumerate(self.trajectories):
            self._traj_dict[t[0].id] = i
    
    @property
    def starts(self) -> dict[int,Pose3D]:
        output = dict()
        for s,traj in self.trajectories:
            output[s.id] = traj.start
        return output
    
    @property
    def ends(self) -> dict[int,Pose3D]:
        output = dict()
        for s,traj in self.trajectories:
            output[s.id] = traj.end
        return output
    
    def poses_at(self,t:float) -> dict[int,Pose3D]:
        output = dict()
        wind_dx = self.wind_x*t
        wind_dy = self.wind_y*t
        
        for s,traj in self.trajectories:
            p = traj.pose_at(t)
            p.x += wind_dx
            p.y += wind_dy
            output[s.id] = p
            
        return output


    def generate_list_of_poses(self,times:list[float]|np.ndarray) -> ListOfTimedPoses:
        output = []
        for t in times:
            poses = self.poses_at(t)
            output.append((t,poses))
        return output
    
    def sample_poses(self,sample_num:int) -> ListOfTimedPoses:
        times = np.linspace(0,self.duration,sample_num,endpoint=False)
        return self.generate_list_of_poses(times)
    
    def sample_at_fps(self,fps:int) -> ListOfTimedPoses:
        return self.sample_poses(np.ceil(self.duration*fps))
    
    def list_ids(self) -> list[int]:
        output = [s.id for s,_ in self.trajectories]
        return output
    
    def generate_id_name_dict(self) -> dict[int,str]:
        output = dict()
        for s,p in self.trajectories:
            output[s.id] = p.abbr()
        return output
    
    def asdict(self) -> dict[str,typing.Any]:
        output = asdict(self)
        output.pop("_traj_dict")
        output.pop("trajectories")
        output["trajectories"] = []
        for t in self.trajectories:
            output["trajectories"].append(
                {
                    "stats" : t[0].asdict(),
                    "path"  : t[1].asdict()
                }
            )
        return output
    
    def join(self,other:typing.Self):
        
        # Check compatibility first
        assert(self.AC_num == other.AC_num)
        assert(self.z_alpha == other.z_alpha)
        assert(self.wind_x == other.wind_x)
        assert(self.wind_y == other.wind_y)
        
        for o_stats,o_traj in other.trajectories:
            s_stats,_ = self.trajectories[self._traj_dict[o_stats.id]]
            assert(s_stats == o_stats)
        
        # Then perform manipulations
        self.separation = min(self.separation,other.separation)
        self.duration += other.duration
        
        for o_stats,o_traj in other.trajectories:
            _,s_traj = self.trajectories[self._traj_dict[o_stats.id]]
            s_traj.join(o_traj)
        
        
            
            
    