# -*- coding: utf-8 -*-
"""
Created on Tue Jan 14 16:23:25 2025

@author: Sagi Amangeldi
"""


#Base libraries
import numpy as np

#3D Libraries
import open3d as o3d


#Define a function that takes as input an array of points, and a voxel size expressed in meters. It returns the sampled point cloud
def grid_subsampling(points, voxel_size):

  nb_vox=np.ceil((np.max(points, axis=0) - np.min(points, axis=0))/voxel_size)
  non_empty_voxel_keys, inverse, nb_pts_per_voxel= np.unique(((points - np.min(points, axis=0)) // voxel_size).astype(int), axis=0, return_inverse=True, return_counts=True)
  idx_pts_vox_sorted=np.argsort(inverse)
  voxel_grid={}
  grid_barycenter,grid_candidate_center=[],[]
  last_seen=0

  for idx,vox in enumerate(non_empty_voxel_keys):
    voxel_grid[tuple(vox)]=points[idx_pts_vox_sorted[last_seen:last_seen+nb_pts_per_voxel[idx]]]
    grid_barycenter.append(np.mean(voxel_grid[tuple(vox)],axis=0))
    grid_candidate_center.append(voxel_grid[tuple(vox)][np.linalg.norm(voxel_grid[tuple(vox)]-np.mean(voxel_grid[tuple(vox)],axis=0),axis=1).argmin()])
    last_seen+=nb_pts_per_voxel[idx]

  return grid_candidate_center


pcd_o3d = o3d.io.read_point_cloud("C:/Users/saman/Desktop/main/data/full_scan.ply")
#Execute the function, and store the results in the grid_sampled_point_cloud variable
grid_sampled_point_cloud = grid_subsampling(pcd_o3d, 15)

#Save the variable to an ASCII file to open in a 3D Software
np.savetxt("C:/Users/saman/Desktop/main/results/automated_sampled.xyz", grid_sampled_point_cloud, delimiter=";", fmt="%s")
