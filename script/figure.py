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
    
    d = 2
    start_degrees = 60
    end_degrees = 330
    
    start   = Pose2D(0,0,np.deg2rad(start_degrees))
    end     = Pose2D(d,0,np.deg2rad(end_degrees))
    
    ## Plotting length variations
    
    step = 0.5
    plot_width = step*0.9
    
    target_lengths = (0,15)
    bins = np.arange(target_lengths[0],target_lengths[1]+step,step)
    centers = (bins[0:-1] + bins[1:])/2
    
    
    radius_samples  = np.linspace(stats.turn_radius,target_lengths[1],3000,True)
    radius_based    = compute_for_different_radius(radius_samples,
                                                start,end)
    
    length_samples = np.linspace(0,target_lengths[1],3000,True)
    both_shift  = compute_for_different_shifts(stats.turn_radius,length_samples,start,end,0.5)
    end_shift   = compute_for_different_shifts(stats.turn_radius,length_samples,start,end,0)
    first_shift = compute_for_different_shifts(stats.turn_radius,length_samples,start,end,1)
    
    list_samples= [length_samples,length_samples,length_samples,radius_samples]
    list_data   = [first_shift,end_shift,both_shift,radius_based]
    
    titles      = ["Shift start", "Shift end", "Shift both", "Turn radius"]
    markers     = [5,4,7,6]
    colors      = [my_cmap6[0],my_cmap6[2],my_cmap6[4], my_cmap6[5]]
                   
    # fig,axes = plt.subplots(2,2,sharey=True)
    
    
    # for i,(samples,values,title) in enumerate(zip(list_samples,
    #                                               list_data,
    #                                                titles)):
    #     ax:Axes = axes[i % 2][i // 2]
    #     if i == 0:
    #         ax.set_xlabel("Radius")
    #     else:
    #         ax.set_xlabel("Straigh extension")

        
    #     ax.plot(samples,values[:,0],label='LRL', color=my_cmap[0])
    #     ax.plot(samples,values[:,1],label='RLR', color=my_cmap[1])
    #     ax.plot(samples,values[:,2],label='LSL', color=my_cmap[2])
    #     ax.plot(samples,values[:,3],label='LSR', color=my_cmap[3])
    #     ax.plot(samples,values[:,4],label='RSL', color=my_cmap[4])
    #     ax.plot(samples,values[:,5],label='RSR', color=my_cmap[5])
    #     ax.plot(samples,values[:,6],label='SLS', color=my_cmap[6])
    #     ax.plot(samples,values[:,7],label='SRS', color=my_cmap[7])
    
    #     ax.set_title(title)
    #     ax.set_xlim(np.min(samples),np.max(samples))
    
    # axes[0][0].set_ylabel("Path length")
    # axes[1][0].set_ylabel("Path length")
    # axes[0][0].legend()
    # fig.tight_layout()
    # fig.suptitle(f"Length fitting for case $\\alpha={start_degrees}°$, $\\beta={end_degrees}°$, $d={d}$")
    # plt.show()
    
    # fig,ax = plt.subplots(figsize=(4,3))
    # fig2,axes2 = plt.subplots(2,4)
    
    # for i,(samples,values,title,marker,color) in enumerate(zip(list_samples,
    #                                                list_data,
    #                                                titles,
    #                                                markers,
    #                                                colors)):
        
    #     LRL_hist,_ = np.histogram(values[:,0],bins,target_lengths)
    #     RLR_hist,_ = np.histogram(values[:,1],bins,target_lengths)
    #     LSL_hist,_ = np.histogram(values[:,2],bins,target_lengths)
    #     LSR_hist,_ = np.histogram(values[:,3],bins,target_lengths)
    #     RSL_hist,_ = np.histogram(values[:,4],bins,target_lengths)
    #     RSR_hist,_ = np.histogram(values[:,5],bins,target_lengths)
    #     SLS_hist,_ = np.histogram(values[:,6],bins,target_lengths)
    #     SRS_hist,_ = np.histogram(values[:,7],bins,target_lengths)
        
        # fig2.suptitle(title)
        # axes2[0][0].bar(centers,LRL_hist,width=plot_width,color=my_cmap[0])
        # axes2[0][0].set_title('LRL')
        # axes2[0][1].bar(centers,RLR_hist,width=plot_width,color=my_cmap[1])
        # axes2[0][1].set_title('RLR')
        # axes2[0][2].bar(centers,LSL_hist,width=plot_width,color=my_cmap[2])
        # axes2[0][2].set_title('LSL')
        # axes2[0][3].bar(centers,LSR_hist,width=plot_width,color=my_cmap[3])
        # axes2[0][3].set_title('LSR')
        # axes2[1][0].bar(centers,RSL_hist,width=plot_width,color=my_cmap[4])
        # axes2[1][0].set_title('RSL')
        # axes2[1][1].bar(centers,RSR_hist,width=plot_width,color=my_cmap[5])
        # axes2[1][1].set_title('RSR')
        # axes2[1][2].bar(centers,SLS_hist,width=plot_width,color=my_cmap[6])
        # axes2[1][2].set_title('SLS')
        # axes2[1][3].bar(centers,SRS_hist,width=plot_width,color=my_cmap[7])
        # axes2[1][3].set_title('SRS')
        # plt.show()
        
        # all_hists = [LRL_hist,RLR_hist,LSL_hist,LSR_hist,RSL_hist,RSR_hist,SLS_hist,SRS_hist]
        
        # possibilites = sum(h > 0 for h in all_hists)
        
        # ax.bar(centers+(i*plot_width/len(titles) - plot_width/2),possibilites,color=color,width=plot_width/len(titles),align='edge',label=title)
        
        # color='0.1'
        # ax.plot(centers,possibilites,color=color,
                # marker=marker,markersize=7,
                # linestyle=(0,(1,4)),
                # label=title)
    
    # ax.set_title(f"Number of length fitted paths for case $\\alpha={start_degrees}°$, $\\beta={end_degrees}°$, $d={d}$ using different methods with resolution {step}")
    # ax.set_xlabel('Target length')
    # ax.set_xlim(bins[0]-step,bins[-1]+step)
    # ax.set_ylabel('Number of possible paths')
    # ax.legend()
    # fig.tight_layout()
    # plt.show()
    
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
    target_length = 10
    
    for name,planner,marker,markersize in [#("LRL",plan_LRL_from_straights),
                         #("RLR",plan_RLR_from_straights), 
                         ("LSL",plan_LSL_from_straights,"o",5),
                         #("LSR",plan_LSR_from_straights), 
                         #("RSL",plan_RSL_from_straights),
                         ("RSR",plan_RSR_from_straights,"+",11), 
                         #("SLS",plan_SLS_from_straights), ("SRS",plan_SRS_from_straights)
                         ]:
        
        color = my_cmap6[j % len(my_cmap6)]
        # color = '0.2'
        j += 2
        
        path = planner(stats,start,end, target_length,1)
        if path is not None:
        
            l = path.total_length
            samples = np.linspace(0,l,100)
            points = []
            
            for i,s in enumerate(samples):
                points.append(path.pose_at(samples[i]))
            
            
            plot_pose2d_sequence(axe,points,False,True,label=f"S-{name}",
                color=color, 
                marker=marker,markersize=markersize,markevery=0.2,
                linestyle='dashed')
            
        path = planner(stats,start,end, target_length,0)
        if path is not None:
        
            l = path.total_length
            samples = np.linspace(0,l,100)
            points = []
            
            for i,s in enumerate(samples):
                points.append(path.pose_at(samples[i]))
            
            
            plot_pose2d_sequence(axe,points,False,True,label=f"{name}-S",
                color=color, 
                marker=marker,markersize=markersize,markevery=0.2,
                linestyle='dotted')
            
        path = planner(stats,start,end, target_length,0.5)
        if path is not None:
        
            l = path.total_length
            samples = np.linspace(0,l,100)
            points = []
            
            for i,s in enumerate(samples):
                points.append(path.pose_at(samples[i]))
            
            
            plot_pose2d_sequence(axe,points,False,True,label=f"S-{name}-S",
                color=color, 
                marker=marker,markersize=markersize,markevery=0.2,
                linestyle='solid')
        
    for spine in axe.spines:
        axe.spines[spine].set_visible(False)
        
    axe.set_xticks([])
    axe.set_yticks([])
    axe.set_aspect('equal')
    axe.legend()
    plt.show()
            
    