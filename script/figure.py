#!/usr/bin/python3

import typing

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.axes import Axes


from Dubins import LRL,RLR,LSL,LSR,RSL,RSR,SLS,SRS,Pose2D,Pose3D,ACStats,Path
from DubinsPathFitting import plan_LRL, plan_RLR, plan_LSL, plan_LSR, plan_RSL, plan_RSR, plan_SLS, plan_SRS,\
        plan_LRL_from_straights, plan_RLR_from_straights, plan_LSL_from_straights, plan_LSR_from_straights,\
        plan_RSL_from_straights, plan_RSR_from_straights, plan_SLS_from_straights, plan_SRS_from_straights
from plotting import plot_pose2d_sequence

from _plotting_extra import my_cmap6,my_cmap
 

def LRL_length(start:Pose2D, end:Pose2D) -> float:
    ans = LRL(start,end)
    if ans is None:
        return np.nan
    else:
        return sum(ans)
    
def RLR_length(start:Pose2D, end:Pose2D) -> float:
    ans = RLR(start,end)
    if ans is None:
        return np.nan
    else:
        return sum(ans)
    
def LSL_length(start:Pose2D, end:Pose2D) -> float:
    ans = LSL(start,end)
    if ans is None:
        return np.nan
    else:
        return sum(ans)
    
def LSR_length(start:Pose2D, end:Pose2D) -> float:
    ans = LSR(start,end)
    if ans is None:
        return np.nan
    else:
        return sum(ans)

def RSL_length(start:Pose2D, end:Pose2D) -> float:
    ans = RSL(start,end)
    if ans is None:
        return np.nan
    else:
        return sum(ans)
    
def RSR_length(start:Pose2D, end:Pose2D) -> float:
    ans = RSR(start,end)
    if ans is None:
        return np.nan
    else:
        return sum(ans)
    
def SLS_length(start:Pose2D, end:Pose2D) -> float:
    ans = SLS(start,end)
    if ans is None:
        return np.nan
    else:
        return sum(ans)
    
def SRS_length(start:Pose2D, end:Pose2D) -> float:
    ans = SRS(start,end)
    if ans is None:
        return np.nan
    else:
        return sum(ans)
    
def all_lengths(start:Pose2D, end:Pose2D) -> np.ndarray:
    return np.array([
        LRL_length(start,end),
        RLR_length(start,end),
        LSL_length(start,end),
        LSR_length(start,end),
        RSL_length(start,end),
        RSR_length(start,end),
        SLS_length(start,end),
        SRS_length(start,end),
    ])
    
def compute_for_different_radius(rs:np.ndarray,start:Pose2D,end:Pose2D) -> np.ndarray:
    output = []
    for r in rs:
        sx = start[0]/r
        sy = start[1]/r
        
        ex = end[0]/r
        ey = end[1]/r
        
        output.append(r*all_lengths(Pose2D(sx,sy,start[2]),Pose2D(ex,ey,end[2])))
    
    return np.asarray(output)

def compute_for_different_shifts(radius:float,lengths:np.ndarray,start:Pose2D,end:Pose2D,ratio:float) -> np.ndarray:
    output = []
    
    vsx = np.cos(start.angle)
    vsy = np.sin(start.angle)
    
    vex = np.cos(end.angle)
    vey = np.sin(end.angle)
    
    
    for l in lengths:
        sx = (start[0] + vsx*l*ratio)/radius
        sy = (start[1] + vsy*l*ratio)/radius
        
        ex = (end[0] - vex*l*(1-ratio))/radius
        ey = (end[1] - vey*l*(1-ratio))/radius
    
        output.append(l+radius*all_lengths(Pose2D(sx,sy,start.angle),Pose2D(ex,ey,end.angle)))
        
    return np.asarray(output)


#################### Plotting ####################

if __name__ == '__main__':
    import argparse
    
    
    stats = ACStats(0,1.,0.,0.5)
    
    d = 8
    start_degrees = 60
    end_degrees = 300
    
    start   = Pose2D(0,0,np.deg2rad(start_degrees))
    end     = Pose2D(d,0,np.deg2rad(end_degrees))
    
    ## Plotting length variations
    
    fig,axes = plt.subplots(2,2,sharey=True)
    
    radius_samples  = np.linspace(stats.turn_radius,5*stats.turn_radius,100,True)
    radius_based    = compute_for_different_radius(radius_samples,
                                                start,end)
    
    length_samples = np.linspace(0,10,100,True)
    both_shift  = compute_for_different_shifts(stats.turn_radius,length_samples,start,end,0.5)
    end_shift   = compute_for_different_shifts(stats.turn_radius,length_samples,start,end,0)
    first_shift = compute_for_different_shifts(stats.turn_radius,length_samples,start,end,1)
    
    list_samples= [radius_samples,length_samples,length_samples,length_samples]
    list_data   = [radius_based,both_shift,first_shift,end_shift]
    titles      = ["Turn radius", "Shift both", "Shift start", "Shift end"]
    
    
    for i,(samples,values,title) in enumerate(zip(list_samples,
                                                  list_data,
                                                   titles)):
        ax:Axes = axes[i % 2][i // 2]
        if i == 0:
            ax.set_xlabel("Radius")
        else:
            ax.set_xlabel("Straigh extension")

        
        ax.plot(samples,values[:,0],label='LRL', color=my_cmap[0])
        ax.plot(samples,values[:,1],label='RLR', color=my_cmap[1])
        ax.plot(samples,values[:,2],label='LSL', color=my_cmap[2])
        ax.plot(samples,values[:,3],label='LSR', color=my_cmap[3])
        ax.plot(samples,values[:,4],label='RSL', color=my_cmap[4])
        ax.plot(samples,values[:,5],label='RSR', color=my_cmap[5])
        ax.plot(samples,values[:,6],label='SLS', color=my_cmap[6])
        ax.plot(samples,values[:,7],label='SRS', color=my_cmap[7])
    
        ax.set_title(title)
        ax.set_xlim(np.min(samples),np.max(samples))
    
    axes[0][0].set_ylabel("Path length")
    axes[1][0].set_ylabel("Path length")
    axes[0][0].legend()
    fig.tight_layout()
    fig.suptitle(f"Length fitting for case $\\alpha={start_degrees}°$, $\\beta={end_degrees}°$, $d={d}$")
    
    
    ## Plotting paths
    
    fig,axe = plt.subplots()
    axe:Axes
    
    
    # for name,planner in [("LRL",plan_LRL), ("RLR",plan_RLR), ("LSL",plan_LSL), ("LSR",plan_LSR), ("RSL",plan_RSL), ("RSR",plan_RSR), ("SLS",plan_SLS), ("SRS",plan_SRS)]:
    #     path = planner(stats,start,end)
    #     if path is None:
    #         continue
        
    #     l = path.total_length
    #     samples = np.linspace(0,l,100)
    #     points = []
        
    #     for i,s in enumerate(samples):
    #         points.append(path.pose_at(samples[i]))
        
        
    #     plot_pose2d_sequence(axe,points,False,True,label=f"{name} : {l:.2f}")
    
    j = 0
    target_length = 12
    
    for name,planner in [("LRL",plan_LRL_from_straights), ("RLR",plan_RLR_from_straights), 
                         ("LSL",plan_LSL_from_straights), ("LSR",plan_LSR_from_straights), 
                         ("RSL",plan_RSL_from_straights), ("RSR",plan_RSR_from_straights), 
                         ("SLS",plan_SLS_from_straights), ("SRS",plan_SRS_from_straights)]:
        
        color = my_cmap6[j % len(my_cmap6)]
        j += 1
        
        path = planner(stats,start,end, target_length,1)
        if path is not None:
        
            l = path.total_length
            samples = np.linspace(0,l,100)
            points = []
            
            for i,s in enumerate(samples):
                points.append(path.pose_at(samples[i]))
            
            
            plot_pose2d_sequence(axe,points,False,True,label=f"S-{name} : {l:.2f}", color=color,linestyle='dashed')
            
        path = planner(stats,start,end, target_length,0)
        if path is not None:
        
            l = path.total_length
            samples = np.linspace(0,l,100)
            points = []
            
            for i,s in enumerate(samples):
                points.append(path.pose_at(samples[i]))
            
            
            plot_pose2d_sequence(axe,points,False,True,label=f"{name}-S : {l:.2f}", color=color,linestyle='dotted')
            
        path = planner(stats,start,end, target_length,0.5)
        if path is not None:
        
            l = path.total_length
            samples = np.linspace(0,l,100)
            points = []
            
            for i,s in enumerate(samples):
                points.append(path.pose_at(samples[i]))
            
            
            plot_pose2d_sequence(axe,points,False,True,label=f"S-{name}-S : {l:.2f}", color=color)
        
    axe.set_aspect('equal')
    axe.legend()
    plt.show()
            
    