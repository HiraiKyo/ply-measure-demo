import numpy as np
from numpy.typing import NDArray
from ply_processor_basics.points.ransac import detect_plane
from ply_processor_basics.points.clustering import plane_clustering
from ply_processor_basics.points.convex_hull import detect_circle
from . import edge

PLANE_THRESHOLD = 0.5

def line_distance(p0, v0, p1, v1):
    v0 = v0 / np.linalg.norm(v0)
    v1 = v1 / np.linalg.norm(v1)

    w0 = p0 - p1
    a = np.cross(v0, v1)
    b = np.linalg.norm(a)

    if b == 0:
        return np.linalg.norm(np.cross(w0, v0))

    return abs(np.dot(w0, a)) / b

def measure(
    points_raw: NDArray[np.floating],
    base_plane_index = 0,
    mil_plane_index = 2,
    circle_plane_index = 4,
    base_expected_edges = 4,
    mil_expected_edges = 4,
    min_plane_points = 1000,
    line_epsilon=5.0
):
    tmp = points_raw

    # Find base plane
    base_plane_indices, base_plane_model = None, None
    for i in range(base_plane_index + 1):
        base_plane_indices, base_plane_model = detect_plane(tmp, threshold=1.0)
        outlier_indices = np.setdiff1d(np.arange(len(tmp)), base_plane_indices)
        tmp = tmp[outlier_indices]
    
    # エッジ検出
    segment_points, line_models = edge.find_edge_segments(points_raw, base_plane_index, expected_edges=base_expected_edges, min_plane_points=min_plane_points, plane_threshold=PLANE_THRESHOLD, epsilon=line_epsilon)

    # ミル部エッジ検出
    mil_segment_points, mil_line_models = edge.find_edge_segments(points_raw, mil_plane_index, expected_edges=mil_expected_edges, min_plane_points=min_plane_points, plane_threshold=PLANE_THRESHOLD, epsilon=line_epsilon)

    # 円筒検出
    # 円筒上面の点群を取得
    tmp = points_raw
    plane_indices = None
    plane_model = None
    outlier_indices = None
    # Select the Nth most large plane
    for i in range(circle_plane_index + 1):
        if outlier_indices is not None:
            tmp = tmp[outlier_indices]
        plane_indices, plane_model = detect_plane(tmp, threshold=PLANE_THRESHOLD)
        outlier_indices = np.setdiff1d(np.arange(len(tmp)), plane_indices)
        
    if plane_model is None or plane_indices is None:
        raise Exception("Failed to detect plane")
    
    # Remove noises by clustering
    clusters = plane_clustering(tmp[plane_indices], eps = 10.0)
    if len(clusters) == 0:
        raise Exception("Failed to remove noise on plane")
    tmp = tmp[plane_indices][clusters[0]]
    
    # Too few points on the plane should be an error
    if len(clusters[0]) < min_plane_points:
        raise Exception(f"Plane points is less than {min_plane_points}")

    # 円盤フィッティング
    indices, center, normal, radius = detect_circle(tmp, plane_model)

    # 各エッジと円筒軸の間の距離を算出
    distances = []
    for line_model in line_models:
        p, v = line_model
        distances.append(line_distance(center, plane_model[:3], p, v))
    # ミルの各エッジと円筒軸の間の距離を算出
    mil_distances = []
    for line_model in mil_line_models:
        p, v = line_model
        mil_distances.append(line_distance(center, plane_model[:3], p, v))
    return center, radius, plane_model[:3], base_plane_indices, segment_points, distances, mil_segment_points, mil_distances

