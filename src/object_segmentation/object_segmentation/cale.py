import open3d as o3d
import numpy as np
cloud = o3d.io.read_point_cloud("points_20251020_184411.ply")
R = cloud.get_rotation_matrix_from_xyz((np.pi, 0, 0))
cloud.rotate(R, center=(0, 0, 0))

o3d.visualization.draw_geometries([cloud])

