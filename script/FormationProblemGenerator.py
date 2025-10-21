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

from Formation import *

from ProblemGenerator import write_pathplanning_problem_to_CSV, AC_PP_Problem, ACStats, Pose3D

def problem_from_formation_move(formation:Formation, move:Pose3D, stats:list[ACStats]) -> AC_PP_Problem:
    assert len(formation.positions) == len(stats)

if __name__ =='__main__':
    import argparse
    
    