# Copyright (C 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of DubinsFleetPlanner.
# 
# DubinsFleetPlanner is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option any later version.
# 
# DubinsFleetPlanner is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

import numpy as np
import json,csv
from dataclasses import dataclass
from enum import IntEnum

class DubinsMove(IntEnum):
    STRAIGHT = 0
    LEFT     = 1
    RIGHT    = 2

@dataclass
class Pose3D:
    x:float # X coordinate
    y:float # Y coordinate
    z:float # Z coordinate
    theta:float # XY orientation, in radian
    
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
    

ListOfTimedPoses = list[tuple[float,list[tuple[int,Pose3D]]]]
DictOfPoseTrajectories = dict[int,list[tuple[float,Pose3D]]]

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