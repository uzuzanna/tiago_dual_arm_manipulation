import open3d as o3d
import numpy as np

cloud = o3d.io.read_point_cloud("points_20251020_184411.ply")
cloud, ind = cloud.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)

points = np.asarray(cloud.points)
points[:, 1] = -points[:, 1]
cloud.points = o3d.utility.Vector3dVector(points)

plane_model, inliers = cloud.segment_plane(distance_threshold=0.003,
                                           ransac_n=3,
                                           num_iterations=1000)

a, b, c, d = plane_model
plane_normal = np.array([a, b, c])
plane_normal /= np.linalg.norm(plane_normal)

dist = (points @ plane_normal) + d
mask_above = dist > 0.0

cloud = o3d.geometry.PointCloud()
cloud.points = o3d.utility.Vector3dVector(points[mask_above])
points = np.asarray(cloud.points)

plane_model, inliers = cloud.segment_plane(distance_threshold=0.003,
                                           ransac_n=3,
                                           num_iterations=1000)
object_cloud = cloud.select_by_index(inliers, invert=True)

points_obj = np.asarray(object_cloud.points)
labels = np.array(object_cloud.cluster_dbscan(eps=0.015, min_points=50))

clusters = [(np.sum(labels == cid), cid) for cid in range(labels.max() + 1)
            if np.sum(labels == cid) > 300]

if not clusters:
    raise RuntimeError("Brak sensownych klastrów")

_, best_id = max(clusters)
points_obj = points_obj[labels == best_id]

z_min = np.percentile(points_obj[:, 2], 5)
z_max = np.percentile(points_obj[:, 2], 90)

mask_z = (points_obj[:, 2] >= z_min) & (points_obj[:, 2] <= z_max)
points_obj = points_obj[mask_z]

object_cloud = o3d.geometry.PointCloud()
object_cloud.points = o3d.utility.Vector3dVector(points_obj)

centroid = points_obj.mean(axis=0)
np.savetxt("centroid.txt", centroid)
print("Centroid:", centroid)

try:
    o3d.visualization.draw_geometries([object_cloud, obb])
except:
    print("(Brak GUI — pominięto wizualizację)")

