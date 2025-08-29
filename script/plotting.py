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

#!/usr/bin/env python3

import numpy as np
import typing
import pathlib

from plotting_extra import linestyle_dict,my_cmap,ColorType
from ioUtils import Pose3D,ListOfTimedPoses,DictOfPoseTrajectories,\
                    parse_trajectories_from_CSV,min_XY_dist,transpose_list_of_trajectories



#################### Plotting ####################

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle,Arrow
from matplotlib.animation import FuncAnimation
from matplotlib.text import Text
from matplotlib import transforms


ARROWS_SCALING = {
    'scale_units':'inches',
    'scale' : 4,
    'units' : 'inches',
    'width' : 0.03
}

def plot_pose2d_sequence(ax:Axes,poses:list[Pose3D],endpoints:bool=False,endarrows:bool=False,**plot_kwargs) -> Axes:
    xs = np.asarray([p.x for p in poses])
    ys = np.asarray([p.y for p in poses])
    angles = np.asarray([p.theta for p in poses])
    
    ax.plot(xs,ys,**plot_kwargs)
    
    # Avoid repeating the name in the legend
    plot_kwargs.pop('label',None)
    plot_kwargs.pop('angles',None)
    plot_kwargs.pop('marker',None)
    
    if (endpoints):
        ax.scatter(xs[0],ys[0],marker='o',**plot_kwargs)
        ax.scatter(xs[-1],ys[-1],marker='X',**plot_kwargs)
    
    
    if (endarrows):
        ax.quiver([xs[0]],[ys[0]],[np.cos(angles[0])],[np.sin(angles[0])],angles='xy',**plot_kwargs)
        ax.quiver([xs[-1]],[ys[-1]],[np.cos(angles[-1])],[np.sin(angles[-1])],angles='xy',**plot_kwargs)
    
    return ax

def plot_several_pose2d_sequences(ax:Axes,poses_dict:DictOfPoseTrajectories,colors_dict:typing.Optional[dict[int,ColorType]]=None,
                                  endpoints:bool=False,endarrows:bool=False,alpha:float=1.) -> Axes:
    i = 0
    for key,values in poses_dict.items():
        pose2d_seq = [v[1] for v in values]
        c = colors_dict[key] if colors_dict is not None else my_cmap[i % len(my_cmap)]
        ax = plot_pose2d_sequence(ax,pose2d_seq,endpoints,endarrows,
                                  color=c,
                                  label=f"AC: {key}",
                                  alpha=alpha)
        
        i += 1
        
        
    return ax


def animate_several_pose2d_sequences(poses_list:ListOfTimedPoses,fps:int=30,
                                     save_animation:typing.Optional[str]=None,
                                     show_animation:bool=False):
    # Set axes aspect
    fig,ax = plt.subplots(figsize=(16,9))
    ax.set_aspect('equal','datalim')
    
    # Sort in ascending time
    poses_list.sort(key=lambda t : t[0])
    
    poses_dict = transpose_list_of_trajectories(poses_list)
    
    # Setup color scheme
    colors_dict = dict()
    i = 0
    for k in poses_dict.keys():
        colors_dict[k] = my_cmap[i % len(my_cmap)]
        i += 1
    
    # Plot background lines
    ax = plot_several_pose2d_sequences(ax,poses_dict,colors_dict,True,False,0.4)
    
    # Setup artists for animation
    init_poses = [t[1] for t in poses_list[0][1]]
    init_xs = [p.x for p in init_poses]
    init_ys = [p.y for p in init_poses]
    init_thetas = [p.theta for p in init_poses]
    init_colors = [colors_dict[t[0]] for t in poses_list[0][1]]
    
    pos_artist = ax.scatter(
        init_xs,init_ys,
        color=init_colors,label="Aircraft positions",marker='+'
    )
    
    dirs_artist = ax.quiver(
        init_xs,init_ys,np.cos(init_thetas),np.sin(init_thetas),
        color=init_colors,label="Aircraft direction",
        angles='xy',**ARROWS_SCALING
    )
    
    min_dist,i1,i2 = min_XY_dist(init_poses)
    id1,p1 = poses_list[0][1][i1]
    id2,p2 = poses_list[0][1][i2]
    
    agent_num_txtwidth = int(np.log10(len(init_poses)))+1
    
    mindist_artist = ax.plot(
        [p1.x,p2.x],
        [p1.y,p2.y],
        marker='D',color='k',linestyle=':',markeredgecolor='r',
        label=f"Min dist ({id1:{agent_num_txtwidth}},{id2:{agent_num_txtwidth}}): {min_dist:.3f}"
    )[0]
    

    # Animation callback function
    def update(frame:int):
        modified_artists = []
        
        poses_and_ids = poses_list[frame][1]
        poses = [t[1] for t in poses_and_ids]
        
        offsets = np.asarray([[p.x,p.y] for p in poses])
        angles = np.asarray([p.theta for p in poses])

        
        
        ### Positions
        
        
        min_dist,i1,i2 = min_XY_dist(poses)
        id1,p1 = poses_and_ids[i1]
        id2,p2 = poses_and_ids[i2]
        
        # Draw only if distance is relevant
        if min_dist < 1e4:
            mindist_artist.set_label(f"Min dist ({id1:{agent_num_txtwidth}},{id2:{agent_num_txtwidth}}): {min_dist:.3f}")
            mindist_artist.set_data([
                [p1.x,p2.x],
                [p1.y,p2.y],
                ])
        else:
            mindist_artist.set_label(f"Min dist ----")
            mindist_artist.set_data([
                [1e9,1e9],
                [1e9,1e9]
                ])
                
        modified_artists.append(mindist_artist)
        
        pos_artist.set_offsets(offsets)
        modified_artists.append(pos_artist)
        
        dirs = np.vstack([np.cos(angles),np.sin(angles)])
        
        dirs_artist.set_offsets(offsets)
        dirs_artist.set_UVC(dirs[0],dirs[1])
        modified_artists.append(dirs_artist)
        
        # wind_artist.set_offsets(poses[:,:2]+dirs.T)
        # modified_artists.append(wind_artist)
        
        # speed_artist.set_offsets(poses[:,:2])
        # speed_artist.set_UVC(dirs[0]+repeated_wind[:,0],dirs[1]+repeated_wind[:,1])
        # modified_artists.append(speed_artist)
        
        modified_artists.append(ax.legend())
        ax.set_title(f"Time: {poses_list[frame][0]:.4f}")
        
        return modified_artists
    
    fig.tight_layout()
    ax.set_xmargin(0.2)
    ax.set_ymargin(0.2)
    ax.legend()
    
    ani = FuncAnimation(fig,update,repeat=True,frames=len(poses_list),interval=1000/fps)
    
    if save_animation is not None:
        print(f"Saving file to '{save_animation}' ...",end='',flush=True)
        ani.save(save_animation,fps=fps)
        print(" Done!")
    
    if show_animation:
        plt.show()


# def plot_dubins_paths(alpha:float,beta:float,d:float,rho:float=1, tol:float=1e-8, samples:int=100):
    
#     start_loc = [0.,0.]
#     end_loc = [0.,d]
    
#     for dclass in dubins_classes:
#         dubins_instance:DubinsPath = dclass((start_loc[0],start_loc[1],alpha),(end_loc[0],end_loc[1],beta),rho)
#         name = dubins_instance.class_base_name()
#         lengths = dubins_instance.sections_lengths()
        
#         total_len = dubins_instance.validated_length(tol)
#         if total_len is None or not(np.isfinite(total_len)):
#             print(f"{name} path is not valid")
#             continue
        
#         print(f"{name} path lengths are {lengths}\nTotal: {total_len}")
#         print(f"Normalized parameters:\n- alpha: {np.rad2deg(dubins_instance.alpha)}\n- beta : {np.rad2deg(dubins_instance.beta)}\n- d    : {dubins_instance.d}")

        
#         poses = dubins_instance.compute_poses(0.,total_len,total_len/samples)
#         plt.plot(poses[:,0],poses[:,1],label=dubins_instance.name(),color=DUBINS_COLORS[name],linestyle=DUBINS_LINESTYLES[name])
        

#     plt.title(f"From {start_loc} to {end_loc} with turning radius {rho}")

#     plt.scatter([start_loc[0]],[start_loc[1]],c='k',marker='X',label='Start')
#     plt.scatter([end_loc[0]],[end_loc[1]],c='k',marker='o',label='End')
#     plt.quiver([start_loc[0]],[start_loc[1]],[np.cos(alpha)],[np.sin(alpha)],angles='xy',label='Start direction')
#     plt.quiver([end_loc[0]],[end_loc[1]],[np.cos(beta)],[np.sin(beta)],angles='xy',label='End direction')
#     plt.legend()
#     plt.gca().set_aspect('equal')
#     plt.show()
    
    

# def animate_dubins(dd:list[DubinsPath],wind:np.ndarray=np.zeros(2,dtype=float),
#                    fps:int=30,
#                    show_distance_matrix:bool=False,
#                    save_animation:typing.Optional[str]=None,
#                    show_animation:bool=True):
    
#     path_num = len(dd)
#     pose_wind = np.array([wind[0],wind[1],0.])
    
#     if path_num > len(my_cmap):
#         this_cmap = np.concatenate([my_cmap]*(1+path_num//len(my_cmap)))
#     else:
#         this_cmap = my_cmap.copy()
    
#     starts = np.array([d.start for d in dd])
#     ends = np.array([d.end+d.total_length()*pose_wind for d in dd])
    
#     if show_distance_matrix:
#         fig,axes = plt.subplots(1,2,figsize=(16,9))
        
#         ax:Axes=axes[0]
#         dist_ax:typing.Optional[Axes] = axes[1]
#     else:
#         fig,ax = plt.subplots(figsize=(16,9))
#         ax:Axes
#         dist_ax = None 
        
#     # Wind arrow, if needed
    
#     if np.linalg.norm(wind) > np.finfo(float).eps * 1e2:
#         wind_dir = np.arctan2(wind[1],wind[0])
#         wind_patch = Arrow(0.15-0.05*np.cos(wind_dir),
#                            0.15-0.05*np.sin(wind_dir),
#                            0.05*np.cos(wind_dir),
#                            0.05*np.sin(wind_dir),
#                            color='0.4',
#                            width=0.02,
#                            edgecolor='k',linewidth=1,
#                            transform=ax.transAxes)
        
#         wind_dir_deg = ((np.rad2deg(wind_dir)-90)+180) % 360
        
#         ax.text(0.15,0.025,
#                      f"Wind: {wind_dir_deg:.1f}° at {np.linalg.norm(wind):.3f} u/s",
#                      va='center',
#                      ha='center',
#                      transform=ax.transAxes)
        
#         ax.add_patch(wind_patch)
    
#     # Plot starting and ending poses    
#     ax.scatter(starts[:,0],starts[:,1],c=this_cmap[:path_num],
#                 label="Starts",marker='o')
    
#     ax.quiver(starts[:,0],starts[:,1],np.cos(starts[:,2]),np.sin(starts[:,2]),#label='Start directions',
#                 edgecolor='k',facecolor=this_cmap[:path_num],linewidth=1,
#                 angles='xy',**ARROWS_SCALING)
    
#     ax.scatter(ends[:,0],ends[:,1],c=this_cmap[:path_num],
#                 label="Ends",marker='X')
    
#     ax.quiver(ends[:,0],ends[:,1],np.cos(ends[:,2]),np.sin(ends[:,2]),label='Ends directions',
#                 edgecolor='k',facecolor=this_cmap[:path_num],linewidth=1,
#                 angles='xy',**ARROWS_SCALING)
    
#     max_l = max(d.total_length() for d in dd)
#     samples = int(max_l * fps)+1
#     all_poses = []
#     max_pos_num = 0
    
#     for i,best_dubins in enumerate(dd):
#         poses = best_dubins.compute_full_path(1/fps,wind=wind)
#         ax.plot(poses[:,0],poses[:,1],label=f"Traj n°{i}, type {best_dubins.name()}, length {best_dubins.total_length():.3f}",
#                 color=this_cmap[i],linestyle='-',alpha=0.5)
#         all_poses.append(poses)
#         max_pos_num = max(max_pos_num,len(poses))
        
#     # When finished, send the aircraft far away to avoid messing with minimal distance computations
#     for i,ps in enumerate(all_poses):
#         if len(ps) < max_pos_num:
#             filler = np.tile((1+i)*1e9*np.ones(ps[-1].shape), (max_pos_num - len(ps),1))
#             all_poses[i] = np.concatenate([ps , filler])
        
    
#     all_poses = np.asarray(all_poses)
    
#     ##### Initial position (and initialise artists) #####
    
#     init_poses = all_poses[:,0]
    
#     ### Distance matrix
#     if show_distance_matrix and dist_ax is not None:
#         distance_matrix = squareform(pdist(init_poses[:,:2],metric='euclidean'))
        
#         distmat_artist = dist_ax.imshow(distance_matrix,
#                                         cmap='magma',
#                                         label="Distance matrix")
#         cbar = dist_ax.figure.colorbar(distmat_artist,ax=dist_ax)
        
        
#         max_dist = np.max(distance_matrix)
#         min_dist = np.inf
#         min_dist_indexes = (len(distance_matrix)+1,len(distance_matrix)+1) # Impossible value as default
#         text_artists = np.empty(distance_matrix.shape,dtype=object)
        
#         for i in range(len(distance_matrix)):
#             for j in range(len(distance_matrix)):
#                 if i == j: continue
#                 d = distance_matrix[i,j]
                
#                 if d < min_dist:
#                     min_dist = d
#                     min_dist_indexes = (i,j)
                    
#                 n_d = d/max_dist
#                 text_artists[i,j] = dist_ax.text(j,i,"Dist: $\\mathbf{"+f"{d:.2f}"+"}$",
#                             ha="center",va="center",
#                             color='w' if n_d < 0.4 else 'k',
#                             fontsize=9)
                
#         ## Highlight the smallest distance
        
#         highlight_sq = Rectangle((min_dist_indexes[0]-0.5,min_dist_indexes[1]-0.5),1,1,
#                                 facecolor=(0.,0.,0.,0.),edgecolor=(1.,0.,0.),linewidth=3.5)
#         highlight_sq_T = Rectangle((min_dist_indexes[1]-0.5,min_dist_indexes[0]-0.5),1,1,
#                                 facecolor=(0.,0.,0.,0.),edgecolor=(1.,0.,0.),linewidth=3.5)
#         square_artist = dist_ax.add_patch(highlight_sq)
#         squareT_artist = dist_ax.add_patch(highlight_sq_T)
    
#     ### Drawing for the aircraft
    
#     pos_artist = ax.scatter(
#         init_poses[:,0],init_poses[:,1],
#         color=this_cmap[:path_num],label="Aircraft positions",marker='+'
#     )
    
#     dirs_artist = ax.quiver(
#         init_poses[:,0],init_poses[:,1],np.cos(init_poses[:,2]),np.sin(init_poses[:,2]),
#         color=this_cmap[:path_num],label="Aircraft direction",
#         angles='xy',**ARROWS_SCALING
#     )
    
#     repeated_wind = np.ones((len(init_poses),2)) * wind
    
#     wind_artist = ax.quiver(
#         init_poses[:,0]+np.cos(init_poses[:,2]),init_poses[:,1]+np.sin(init_poses[:,2]),
#         repeated_wind[:,0], repeated_wind[:,1],
#         angles='xy', color='0.4',label=f"Wind: {wind}",
#         **ARROWS_SCALING
#     )
    
#     speed_artist = ax.quiver(
#         init_poses[:,0],init_poses[:,1],
#         np.cos(init_poses[:,2])+repeated_wind[:,0],np.sin(init_poses[:,2])+repeated_wind[:,1],
#         angles='xy',color='k',label=f"Ground speed",
#         scale_units='xy',scale=1,units='inches',width=0.04,
#     )
    
#     min_dist,min_dist_locs = min_sep(init_poses[:,:2])
    
#     agent_num_txtwidth = int(np.log10(len(dd)))+1
    
#     mindist_artist = ax.plot(
#         [init_poses[min_dist_locs[0]][0],init_poses[min_dist_locs[1]][0]],
#         [init_poses[min_dist_locs[0]][1],init_poses[min_dist_locs[1]][1]],
#         marker='D',color='k',linestyle=':',markeredgecolor='r',
#         label=f"Min dist ({min_dist_locs[0]:{agent_num_txtwidth}},{min_dist_locs[1]:{agent_num_txtwidth}}): {min_dist:.3f}"
#     )[0]
    
    
#     ax.set_aspect('equal','datalim')
    
#     def update(frame):
#         modified_artists = []
        
#         poses = all_poses[:,frame]
        
#         ### Distances
#         if show_distance_matrix:
#             distance_matrix = squareform(pdist(poses[:,:2],metric='euclidean'))
#             distmat_artist.set_data(distance_matrix)
            
#             min_dist = np.inf
#             min_dist_indexes = None
            
#             for i in range(len(distance_matrix)):
#                 for j in range(i+1,len(distance_matrix)):
#                     d = distance_matrix[i,j]
                    
#                     if d < min_dist:
#                         min_dist = d
#                         min_dist_indexes = (i,j)
                        
#                     n_d = d/max_dist
#                     txt_artist:Text = text_artists[i,j]
#                     txt_artist.set_text("Dist: $\\mathbf{"+f"{d:.2f}"+"}$")
#                     txt_artist.set_color('w' if n_d < 0.4 else 'k')
#                     modified_artists.append(txt_artist)
                    
#                     txt_artist:Text = text_artists[j,i]
#                     txt_artist.set_text("Dist: $\\mathbf{"+f"{d:.2f}"+"}$")
#                     txt_artist.set_color('w' if n_d < 0.4 else 'k')
#                     modified_artists.append(txt_artist)
                    
#             ## Highlight the smallest distance
#             square_artist.set_xy((min_dist_indexes[0]-0.5,min_dist_indexes[1]-0.5))
#             squareT_artist.set_xy((min_dist_indexes[1]-0.5,min_dist_indexes[0]-0.5))
#             modified_artists.append(square_artist)
#             modified_artists.append(squareT_artist)
            
            
            
#         ### Positions
        
#         # pos_artist.set_offsets(poses[:,:2])
        
#         min_dist,min_dist_locs = min_sep(poses[:,:2])
        
#         # Draw only if distance is relevant
#         if min_dist < 1e4:
#             mindist_artist.set_label(f"Min dist ({min_dist_locs[0]:{agent_num_txtwidth}},{min_dist_locs[1]:{agent_num_txtwidth}}): {min_dist:.3f}")
#             mindist_artist.set_data([
#                 [poses[min_dist_locs[0]][0],poses[min_dist_locs[1]][0]],
#                 [poses[min_dist_locs[0]][1],poses[min_dist_locs[1]][1]]
#                 ])
#         else:
#             mindist_artist.set_label(f"Min dist ----")
#             mindist_artist.set_data([
#                 [1e9,1e9],
#                 [1e9,1e9]
#                 ])
                
#         modified_artists.append(mindist_artist)
        
#         pos_artist.set_offsets(poses[:,:2])
#         modified_artists.append(pos_artist)
        
#         dirs = np.vstack([np.cos(poses[:,2]),np.sin(poses[:,2])])
        
#         dirs_artist.set_offsets(poses[:,:2])
#         dirs_artist.set_UVC(dirs[0],dirs[1])
#         modified_artists.append(dirs_artist)
        
#         wind_artist.set_offsets(poses[:,:2]+dirs.T)
#         modified_artists.append(wind_artist)
        
#         speed_artist.set_offsets(poses[:,:2])
#         speed_artist.set_UVC(dirs[0]+repeated_wind[:,0],dirs[1]+repeated_wind[:,1])
#         modified_artists.append(speed_artist)
        
#         modified_artists.append(ax.legend())
#         ax.set_title(f"Length traveled: {max_l*frame/np.shape(all_poses)[1]:.4f}")
        
#         return modified_artists
    
#     fig.tight_layout()
#     ax.set_xmargin(0.2)
#     ax.set_ymargin(0.2)
    
    
    
#     ani = FuncAnimation(fig,update,repeat=True,frames=samples,interval=1000/fps)
    
#     if save_animation is not None:
#         print(f"Saving file to '{save_animation}' ...",end='',flush=True)
#         ani.save(save_animation,fps=fps)
#         print(" Done!")
    
#     if show_animation:
#         plt.show()
        

# def draw_separation_matrix(dubins_list:list[DubinsPath],ax:typing.Optional[Axes]=None) -> Axes:
#     distance_matrix,loc_matrix,min_dist,min_dist_loc,min_dist_indexes = dubins_distances_matrix(tuple(dubins_list))
            
#     max_dist = np.max(distance_matrix[np.isfinite(distance_matrix)])
    
#     if ax is None:
#         _,ax = plt.subplots(figsize=(16,9))
    
#     im = ax.imshow(distance_matrix,
#                             cmap='magma',
#                             label="Distance matrix")
#     _ = ax.figure.colorbar(im,ax=ax)
    
#     for i in range(len(loc_matrix)):
#         for j in range(len(loc_matrix)):
#             if i == j: continue
#             d = distance_matrix[i,j]
#             n_d = (d - min_dist)/max_dist
#             ax.text(j,i,"Dist: $\\mathbf{"+f"{d:.2f}"+"}$"+f"\nLoc : {loc_matrix[i,j]:.2f}",
#                             ha="center",va="center",
#                             color='w' if n_d < 0.4 else 'k',
#                             fontsize=9)
            
#     highlight_sq = Rectangle((min_dist_indexes[0]-0.5,min_dist_indexes[1]-0.5),1,1,
#                                 facecolor=(0.,0.,0.,0.),edgecolor=(1.,0.,0.),linewidth=3.5)
#     highlight_sq_T = Rectangle((min_dist_indexes[1]-0.5,min_dist_indexes[0]-0.5),1,1,
#                                 facecolor=(0.,0.,0.,0.),edgecolor=(1.,0.,0.),linewidth=3.5)
#     ax.add_patch(highlight_sq)
#     ax.add_patch(highlight_sq_T)
    
#     ax.set_title("Minimal distance matrix between paths")
#     ax.legend()
    
#     return ax
    

# def draw_dubins(dubins_list:list[DubinsPath],
#                 wind:np.ndarray, 
#                 samples:int, 
#                 initial_sols:typing.Optional[list[DubinsPath]]=None,
#                 show_intermediate:typing.Iterable[float]=[],
#                 ax:typing.Optional[Axes]=None) -> Axes:
    
#     if ax is None:
#         _,ax = plt.subplots(figsize=(16,9))
    
    
#     if len(dubins_list) > len(my_cmap):
#         this_cmap = np.concatenate([my_cmap]*(1+len(dubins_list)//len(my_cmap)))
#     else:
#         this_cmap = my_cmap.copy()
    
#     pose_wind = np.array([wind[0],wind[1],0])
    
#     starts = np.array([d.start for d in dubins_list])
#     ends = np.array([d.end+d.total_length()*pose_wind for d in dubins_list])
    
#     if initial_sols is not None: 
#         for i,dubin in enumerate(initial_sols):
#             l = dubin.validated_length()
#             if l is None: continue
#             poses = dubin.compute_poses(0.,l,l/samples)
#             ax.plot(poses[:,0],poses[:,1],label=f"Traj n°{i} (length: {l:.3f} , no wind)",color=my_cmap[i%len(my_cmap)],
#                             linestyle=linestyle_dict['loosely dashed'])
                
        
        
#     for i,best_dubins in enumerate(dubins_list):
#         poses = best_dubins.compute_full_path(best_dubins.total_length()/samples,wind)
#         ax.plot(poses[:,0],poses[:,1],label=f"N°{i}, {best_dubins.name()}, length {best_dubins.total_length():.3f}",
#                     color=my_cmap[i % len(my_cmap)],linestyle='-')
        

#     distance_matrix,loc_matrix,min_dist,min_dist_loc,min_dist_indexes = dubins_distances_matrix(tuple(dubins_list))
            
#     max_dist = np.max(distance_matrix[np.isfinite(distance_matrix)]),
#     min_pos1 = dubins_list[min_dist_indexes[0]].compute_pose_at(min_dist_loc,wind)
#     min_pos2 = dubins_list[min_dist_indexes[1]].compute_pose_at(min_dist_loc,wind)
            
#     ax.plot([min_pos1[0],min_pos2[0]],
#                     [min_pos1[1],min_pos2[1]],
#                     label=f"Closest points (dist: {min_dist:.3f})",
#                     color='k',
#                     linestyle=':',
#                     marker='D',markeredgecolor='r')

        
#     # Plot starting and ending poses
#     if initial_sols is not None:
#         ax.scatter(starts[:,0],starts[:,1],c=this_cmap[:len(initial_sols)],
#                         label="Starts",marker='o')
        
#         ax.scatter(ends[:,0],ends[:,1],c=this_cmap[:len(initial_sols)],
#                         label="Ends",marker='X')
        
#     ax.quiver(starts[:,0],starts[:,1],np.cos(starts[:,2]),np.sin(starts[:,2]),angles='xy',label='Start directions',
#                     color='k',
#                     **ARROWS_SCALING)
        
    
#     ax.quiver(ends[:,0],ends[:,1],np.cos(ends[:,2]),np.sin(ends[:,2]),angles='xy',label='End directions',
#                     color='k',edgecolor='r',linewidth=2,
#                     **ARROWS_SCALING)
    
#     # Wind arrow, if needed
    
#     if np.linalg.norm(wind) > np.finfo(float).eps * 1e2:
#         wind_dir = np.arctan2(wind[1],wind[0])
#         wind_patch = Arrow(0.15-0.05*np.cos(wind_dir),
#                            0.15-0.05*np.sin(wind_dir),
#                            0.05*np.cos(wind_dir),
#                            0.05*np.sin(wind_dir),
#                            color='0.4',
#                            width=0.02,
#                            edgecolor='k',linewidth=1,
#                            transform=ax.transAxes)
        
#         wind_dir_deg = ((np.rad2deg(wind_dir)-90)+180) % 360
        
#         ax.text(0.15,0.025,
#                      f"Wind: {wind_dir_deg:.1f}° at {np.linalg.norm(wind):.3f} u/s",
#                      va='center',
#                      ha='center',
#                      transform=ax.transAxes)
        
#         ax.add_patch(wind_patch)
    
    
#     # Intermediary values, if needed
#     for p in show_intermediate:
#         if p <= 0 or p >= 1:
#             continue
        
        
#         for i,best_dubins in enumerate(dubins_list):
#             l = best_dubins.total_length()
#             pose = best_dubins.compute_pose_at(p*l,wind)
            
#             ax.scatter(pose[0],pose[1],alpha=0.5,
#                         color=my_cmap[i % len(my_cmap)],linestyle='-')
#             ax.quiver(pose[0],pose[1],np.cos(pose[2]),np.sin(pose[2]),angles='xy',
#                 color=my_cmap[i % len(my_cmap)],
#                 alpha=0.5,edgecolor='k',linewidth=1,
#                 **ARROWS_SCALING)
    
#     # Titles and legends
            
#     ax.legend()
        
#     ax.set_aspect('equal','datalim')

#     return ax
        
        


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser('Dubins plotter','Display a Dubins path planning result.')
    parser.add_argument('file',type=str,help='File from which to parse data')
    parser.add_argument('save',type=str,nargs='?',default=None,help='Optional argument: if set, specifies where to save the animation')
    parser.add_argument('-p','--print',action='store_true',help='Show resulting animation if set')
    parser.add_argument('--fps',type=int,help="Number of Frames Per Second when building animation. Default is 30",default=30)
    
    args = parser.parse_args()
    
    path = pathlib.Path(args.file)
    save = args.save
    show_anim = args.print
    
    if not(path.is_file()):
        print(f"{path} is not a file!! Exiting....")
        exit(1)
        
    ext = path.suffix
    
    if ext.lower() == '.csv':
        print("Parsing...",end='')
        data = parse_trajectories_from_CSV(path)
        print(" Done!\nCreating animation...")
        animate_several_pose2d_sequences(data,args.fps,save,show_anim)
        print("Done! Exiting...")
    else:
        print(f"File extension not handled: {ext}\nExiting...")
        