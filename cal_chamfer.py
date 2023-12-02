import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree as KDTree


def compute_chamfer(gt_points, points):
    """
    This function computes a symmetric chamfer distance, i.e. the sum of both chamfers.
  
    """

    gt_points_np = gt_points

    # one direction
    gen_points_kd_tree = KDTree(points)
    one_distances, one_vertex_ids = gen_points_kd_tree.query(gt_points_np)

    recall = sum(i<0.1 for i in one_distances)
    recall = recall/one_distances.shape

    gt_to_gen_chamfer = np.mean(np.square(one_distances))

    # other direction
    gt_points_kd_tree = KDTree(gt_points_np)
    two_distances, two_vertex_ids = gt_points_kd_tree.query(points)
    gen_to_gt_chamfer = np.mean(np.square(two_distances))

    # return gt_to_gen_chamfer
    return gt_to_gen_chamfer + gen_to_gt_chamfer,recall

if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("./mapping.pcd")
    gtpcd = o3d.io.read_point_cloud("./gt.pcd")

    result,recall = compute_chamfer(gtpcd.points, pcd.points)
    print(result,recall)
