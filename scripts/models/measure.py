from typing import List, Tuple
import open3d as o3d
import numpy as np
from numpy.typing import NDArray
from ply_processor_basics.points.convex_hull import detect_line
from ply_processor_basics.points.ransac import detect_plane
from ply_processor_basics.points.clustering import plane_clustering
from ply_processor_basics.points.convex_hull import detect_circle

THRESH = 0.5

def line_distance(p0, v0, p1, v1):
    v0 = v0 / np.linalg.norm(v0)
    v1 = v1 / np.linalg.norm(v1)

    w0 = p0 - p1
    a = np.cross(v0, v1)
    b = np.linalg.norm(a)

    if b == 0:
        return np.linalg.norm(np.cross(w0, v0))

    return abs(np.dot(w0, a)) / b

def measure(points: NDArray[np.floating]):
    line_segment_points = (np.empty((0, 3)), np.empty((0, 3)))

    # エッジ検出
    base_plane_indices, base_plane_model = detect_plane(points, threshold=0.5)
    if base_plane_model is None or base_plane_indices is None:
        raise Exception("Failed to detect plane")
    indices, line_segments_indices, line_models = detect_line(points[base_plane_indices], base_plane_model)
    for i in range(len(line_segments_indices)):
        line_segment_points = (
            np.concatenate([line_segment_points[0], points[base_plane_indices][line_segments_indices[0][i]]]),
            np.concatenate([line_segment_points[1], points[base_plane_indices][line_segments_indices[1][i]]])
        )
    line_segment_points, line_models = reorder_segment_points(line_segment_points, line_models)
    line_segment_points, line_models = remove_short_edges(line_segment_points, line_models, threshold=10.0)

    # ミル部エッジ検出
    mil_line_segment_points = (np.empty((0, 3)), np.empty((0, 3)))
    outlier_indices = np.setdiff1d(np.arange(len(points)), base_plane_indices)
    tmp = points[outlier_indices]
    mil_plane_indices, mil_plane_model = detect_plane(tmp, threshold=0.5)
    if mil_plane_model is None or mil_plane_indices is None:
        raise Exception("Failed to detect mil plane")
    mil_indices, mil_line_segments_indices, mil_line_models = detect_line(tmp[mil_plane_indices], mil_plane_model)
    for i in range(len(mil_line_segments_indices)):
        mil_line_segment_points = (
            np.concatenate([line_segment_points[0], tmp[mil_plane_indices][mil_line_segments_indices[0][i]]]),
            np.concatenate([line_segment_points[1], tmp[mil_plane_indices][mil_line_segments_indices[1][i]]])
        )
    mil_line_segment_points, mil_line_models = reorder_segment_points(mil_line_segment_points, mil_line_models)
    mil_line_segment_points, mil_line_models = remove_short_edges(mil_line_segment_points, mil_line_models, threshold=10.0)

    # line_segments_pointsのうち、

    # エッジを可視化
    # line_set = o3d.geometry.LineSet()
    # line_set.points = o3d.utility.Vector3dVector(points[base_plane_indices])
    # line_set.lines = o3d.utility.Vector2iVector(line_segments_indices)
    # o3d.visualization.draw_geometries([line_set], window_name="Detected Edge")

    # 円筒検出
    # 円筒上面の点群を取得
    loops = 2
    tmp = points
    plane_indices, plane_model = base_plane_indices, base_plane_model
    for i in range(loops):
        outlier_indices = np.setdiff1d(np.arange(len(tmp)), plane_indices)
        tmp = tmp[outlier_indices]
        plane_indices, plane_model = detect_plane(tmp, threshold=1.0)

    # クラスタリングを行う
    clusters = plane_clustering(tmp[plane_indices], eps = 10.0)
    if len(clusters) == 0:
        raise Exception("Failed to detect cylinder top")
    points_cylinder_top = tmp[plane_indices][clusters[0]]
    # visualize(points_cylinder_top)

    # 円盤フィッティング
    indices, center, normal, radius = detect_circle(points_cylinder_top, plane_model)

    # 各エッジと円筒軸の間の距離を算出
    distances = []
    for line_model in line_models:
        p, v = line_model
        distances.append(line_distance(center, normal, p, v))
    # ミルの各エッジと円筒軸の間の距離を算出
    mil_distances = []
    for line_model in mil_line_models:
        p, v = line_model
        mil_distances.append(line_distance(center, normal, p, v))
    return center, radius, normal, base_plane_indices, line_segment_points, distances, mil_line_segment_points, mil_distances


def find_min_point(points: List[List[float]]) -> List[float]:
    """
    点群の中で最小の点を探す
    """
    return min(points, key=lambda x: (x[0], x[1], x[2]))

def reorder_segment_points(
    segment_points,
    line_models
):
    """
    線分セグメントを、最小点から始まるように並べ替える
    """
    min_point = find_min_point(segment_points[0])
    min_index = segment_points[0].index(min_point)
    new_segment_points = (
        segment_points[0][min_index:] + segment_points[0][:min_index],
        segment_points[1][min_index:] + segment_points[1][:min_index]
    )
    new_line_models = line_models[min_index:] + line_models[:min_index]
    return new_segment_points, new_line_models

def remove_short_edges(
    segment_points,
    line_models,
    threshold: float
):
    """
    短いエッジを取り除く
    """
    new_segment_points = (np.empty((0, 3)), np.empty((0, 3)))
    new_line_models = []
    for i in range(len(segment_points[0])):
        if np.linalg.norm(np.array(segment_points[0][i]) - np.array(segment_points[1][i])) > threshold:
            new_segment_points = (np.concatenate([new_segment_points[0], segment_points[0][i]]), np.concatenate([new_segment_points[1], segment_points[1][i]]))
            new_line_models.append(line_models[i])
    return new_segment_points, new_line_models