import open3d as o3d
import numpy as np
import time
from python_app_utils.log import Logger

IMAGE_NAME = "out.png"

def generate_disk_mesh(center, radius, normal):
  # 円盤とエッジを描画
  mesh_disk = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=3.0, resolution=50, split=1)
  mesh_disk.compute_vertex_normals()
  normal = normal / np.linalg.norm(normal)
  rotation_axis = np.cross(np.array([0, 0, 1]), normal)
  rotation_angle = np.arccos(np.dot(np.array([0, 0, 1]), normal))
  rotation_matrix = mesh_disk.get_rotation_matrix_from_axis_angle(rotation_axis * rotation_angle)
  mesh_disk.rotate(rotation_matrix, center=(0, 0, 0))
  mesh_disk.translate(center)
  return mesh_disk

def generate_edge_line(points, plane_indices, line_segments_indices):
  line_set = o3d.geometry.LineSet()
  line_set.points = o3d.utility.Vector3dVector(points[plane_indices])
  line_set.lines = o3d.utility.Vector2iVector(line_segments_indices)
  return line_set

def capture_image(pcds, outdir, cam_front, cam_lookat, cam_up, cam_zoom):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)

    for pcd in pcds:
        vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.show_coordinate_frame = True
    opt.mesh_show_back_face = True
    opt.mesh_show_wireframe = True  # メッシュのワイヤーフレームを表示
    opt.background_color = np.asarray([1, 1, 1])  # 背景色を設定

    # Zoom, front, lookat, upの設定
    ctr = vis.get_view_control()
    ctr.set_zoom(cam_zoom)
    ctr.set_front(cam_front)
    ctr.set_lookat(cam_lookat)
    ctr.set_up(cam_up)

    for pcd in pcds:
        vis.update_geometry(pcd)

    # 画像保存
    vis.poll_events()
    vis.update_renderer()
    time.sleep(1)
    outpath = f"{outdir}/{IMAGE_NAME}"
    vis.capture_screen_image(outpath, do_render=True)

    logger = Logger()
    logger.info(f"Captured snapshot: {outdir}/{IMAGE_NAME}")

    return outpath