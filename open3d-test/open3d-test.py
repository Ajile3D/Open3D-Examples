import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import sys

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

# read in the point cloud passed from the command line
if len(sys.argv) < 2:
    filename = "pointCloud_0.pcd"
else:
    filename = sys.argv[1]
pcd = o3d.io.read_point_cloud(filename)
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])

# downsample
print("Downsample the point cloud with a voxel size of 1.0")
downpcd = pcd.voxel_down_sample(voxel_size=1)
o3d.visualization.draw_geometries([downpcd])

# plane segmentation
plane_model, inliers = downpcd.segment_plane(distance_threshold=1,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = downpcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = downpcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

# crop point cloud based on plane equation
print("Load a polygon volume and use it to crop the original point cloud")
bounding = inlier_cloud.get_oriented_bounding_box()
#bounding, _ = inlier_cloud.compute_convex_hull()
vol = o3d.visualization.SelectionPolygonVolume()
vol.orthogonal_axis = "Z"
vol.bounding_polygon = o3d.utility.Vector3dVector(bounding.get_box_points())
vol.axis_max = bounding.get_min_bound()[2]
vol.axis_min = 0
print(np.asarray(vol.bounding_polygon))
#vol = plane_model
#print(type(plane_model))
pcd = vol.crop_point_cloud(pcd)
# also see pcd.crop(bounding_box) to simply crop around a bounding box
o3d.visualization.draw_geometries([pcd])

# SOR
print("Statistical oulier removal")
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20,
                                         std_ratio=2.0)
display_inlier_outlier(pcd, ind)
pcd = pcd.select_by_index(ind))

# downsample point cloud
downpcd = pcd.voxel_down_sample(voxel_size=1)
o3d.visualization.draw_geometries([downpcd])

# boudning box
aabb = pcd.get_axis_aligned_bounding_box()
aabb.color = (1,0,0)
obb = pcd.get_oriented_bounding_box()
obb.color = (0,1,0)
o3d.visualization.draw_geometries([pcd, aabb, obb])

# convex hull
hull, _ = downpcd.compute_convex_hull()
hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
hull_ls.paint_uniform_color((1, 0, 0))
o3d.visualization.draw_geometries([downpcd, hull_ls])

# DBSCAN clustering
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd.cluster_dbscan(eps=1, min_points=30, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
segmentedPcd = o3d.geometry.PointCloud(pcd)
segmentedPcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([segmentedPcd])

# remove all but the biggest cluster
largestLabel = -1
largestSize = 0
largestCluster = []
for label in range(labels.max()):
    indices = np.where(labels==label)[0]
    if len(indices) > largestSize:
        largestLabel = label
        largestCluster = indices
        largestSize = len(indices)
pcd = pcd.select_by_index(largestCluster)
o3d.visualization.draw_geometries([pcd])

# compute normals, needed for surfacing
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

# surface with alpha shapes
print("Run Alpha Shapes surface reconstruction.")
alpha = 1.0
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# ball pivoting reconstruction
print("Run ball pivoting surface reconstruction.")
radii = [1]
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
downpcd = pcd.voxel_down_sample(voxel_size=1)
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
               downpcd, o3d.utility.DoubleVector(radii))
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([rec_mesh], mesh_show_back_face=True)

# poisson reconstruction
print('Run Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
print(mesh)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# Poisson reconstruction has too many false triangles. We use the density map to eliminate these triangles
print('visualize densities')
densities = np.asarray(densities)
density_colors = plt.get_cmap('plasma')(
    (densities - densities.min()) / (densities.max() - densities.min()))
density_colors = density_colors[:, :3]
density_mesh = o3d.geometry.TriangleMesh()
density_mesh.vertices = mesh.vertices
density_mesh.triangles = mesh.triangles
density_mesh.triangle_normals = mesh.triangle_normals
density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([density_mesh])

print('remove low density vertices')
vertices_to_remove = densities < np.quantile(densities, 0.2)
mesh.remove_vertices_by_mask(vertices_to_remove)
print(mesh)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

o3d.io.write_triangle_mesh("mesh.ply", mesh)
