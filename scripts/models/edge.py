import numpy as np
from ply_processor_basics.points.convex_hull import detect_line
from ply_processor_basics.points.ransac import detect_plane
from ply_processor_basics.points.clustering import plane_clustering

def find_edge_segments(points, plane_index, expected_edges=4, plane_threshold=0.5, min_plane_points = 1000, epsilon=5):
    line_segment_points = (np.empty((0, 3)), np.empty((0, 3)))
    tmp = points
    plane_indices = None
    plane_model = None
    outlier_indices = None
    # Select the Nth most large plane
    for i in range(plane_index + 1):
        if outlier_indices is not None:
            tmp = tmp[outlier_indices]
        plane_indices, plane_model = detect_plane(tmp, threshold=plane_threshold)
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
    
    # Detect edge line
    indices, line_segments_indices, line_models = detect_line(tmp, plane_model, epsilon=epsilon)
    for i in range(len(line_segments_indices)):
        line_segment_points = (
            np.concatenate([line_segment_points[0], np.asarray([tmp[line_segments_indices[i][0]]])]),
            np.concatenate([line_segment_points[1], np.asarray([tmp[line_segments_indices[i][1]]])])
        )

    # Postprocess: find 4 most longest edges, ordered by the length
    line_segment_points, line_models = reorder_segment_points(line_segment_points, line_models)
    line_segment_points, line_models = pick_largest_segments(line_segment_points, line_models, amount=expected_edges)
    return line_segment_points, line_models

def find_min_point(points):
    """
    点群の中で最小の点を探す
    """
    index, min_point = min(enumerate(points), key=lambda x: x[1][0] + x[1][1] + x[1][2])
    return index, min_point

def reorder_segment_points(
    segment_points,
    line_models
):
    """
    線分セグメントを、最小点から始まるように並べ替える
    """
    min_index, min_point = find_min_point(segment_points[0])
    new_segment_points = (
        np.concatenate([segment_points[0][min_index:], segment_points[0][:min_index]]),
        np.concatenate([segment_points[1][min_index:], segment_points[1][:min_index]])
    )
    new_line_models = line_models[min_index:] + line_models[:min_index]
    
    # Clockwise reorder
    relative_vectors = new_segment_points[0] - new_segment_points[0][0]
    normal = np.cross(relative_vectors[1] - relative_vectors[0], relative_vectors[2] - relative_vectors[0])
    if normal[2] < 0:
        normal = -normal
    
    cross_products = np.cross(relative_vectors, normal)
    sorted_indices = np.argsort(cross_products[:, 2])
    ordered_vertices = new_segment_points[0][sorted_indices]

    if np.all(new_segment_points[0][1] != ordered_vertices[1]):
        new_segment_points = (
            new_segment_points[1][::-1],
            new_segment_points[0][::-1]
        )
        new_line_models = new_line_models[::-1]
    return new_segment_points, new_line_models

def cal_edge_length(start, end):
    return np.linalg.norm(end - start)

def pick_largest_segments(segment_points, line_models, amount=4):
    """
    Pick largest segments, keeping order
    """
    print("BEFORE")
    print(segment_points)
    edge_lengths = []
    for i in range(len(segment_points[0])):
        length = cal_edge_length(segment_points[0][i], segment_points[1][i])
        edge_lengths.append((i, length))
    ordered_lengths = sorted(edge_lengths, key=lambda x: x[1], reverse=True)[:amount]
    print("LENGTHS")
    print(edge_lengths)
    print("ORDERED")
    print(ordered_lengths)
    picked_segments = (
        [segment_points[0][i] for i, _ in ordered_lengths],
        [segment_points[1][i] for i, _ in ordered_lengths]
    )
    picked_line_models = [line_models[i] for i, _ in ordered_lengths]
    print("AFTER")
    print(picked_segments)
    return picked_segments, picked_line_models

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
    print(len(segment_points[0]))
    print(len(line_models))
    for i in range(len(segment_points[0])):
        if np.linalg.norm(np.array(segment_points[0][i]) - np.array(segment_points[1][i])) > threshold:
            new_segment_points = (
                np.concatenate([new_segment_points[0], np.asarray([segment_points[0][i]])]),
                np.concatenate([new_segment_points[1], np.asarray([segment_points[1][i]])])
            )
            new_line_models.append(line_models[i])
    return new_segment_points, new_line_models