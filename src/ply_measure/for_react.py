from typing import List, Tuple, Union
from pydantic import BaseModel
import open3d as o3d
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import eel

from eel_ros1 import actions_util

from .models import measure, gen_points, visualize
from .utils import config, pc2

class DistanceSet(BaseModel):
  distance: float
  line_segment_points: Tuple[List[float], List[float]]
  group: int
  image_path: Union[str, None]

class MeasureResult(BaseModel):
  center: List[float]
  radius: float
  normal: List[float]
  distances: List[DistanceSet]
  plane_indices: List[int]

last_points = None
cfg = config.Config()

# Publishers
pub_result = rospy.Publisher(cfg.ROS_PUB_TOPIC_RESULT, String, queue_size=1)
pub_pointcloud = rospy.Publisher(cfg.ROS_PUB_TOPIC_POINTCLOUD, PointCloud2, queue_size=1)
pub_image = rospy.Publisher(cfg.ROS_PUB_TOPIC_IMAGE, Image, queue_size=1)

@eel.expose
def take_snapshot():
    global last_points
    filepath = actions_util.open_filebrowser()
    pcd = o3d.io.read_point_cloud(filepath)
    points = np.asarray(pcd.points)
    last_points = points
    # pub_pointcloud.publish(pc2.numpy_to_pc2(points))

@eel.expose
def process_pointcloud():
    global last_points
    points = last_points

    # ply-processor-basicsを利用して円柱中心とエッジを検出
    center, radius, normal, plane_indices, line_segment_points, distances, mil_line_segment_points, mil_distances = measure.measure(
        points,
        base_plane_index=cfg.BASE_PLANE_INDEX,
        mil_plane_index=cfg.MIL_PLANE_INDEX,
        circle_plane_index=cfg.CIRCLE_PLANE_INDEX,
        base_expected_edges=cfg.BASE_EXPECTED_EDGES,
        mil_expected_edges=cfg.MIL_EXPECTED_EDGES,
        min_plane_points=cfg.MIN_PLANE_POINTS,
        line_epsilon=cfg.CONVEX_HULL_EPSILON
    )

    # Datastore層に結果を保存(->FluxパターンでGUI更新)
    distanceSets = [DistanceSet(
        distance=d,
        group=0,
        line_segment_points=(line_segment_points[0][i], line_segment_points[1][i]),
        image_path=None
    ) for i, d in enumerate(distances)]
    distanceSets += [DistanceSet(
        distance=d,
        group=1,
        line_segment_points=(mil_line_segment_points[0][i], mil_line_segment_points[1][i]),
        image_path=None
    ) for i, d in enumerate(mil_distances)]

    result = MeasureResult(
      center=center,
      radius=radius,
      normal=normal,
      distances=distanceSets,
      plane_indices=plane_indices,
    )

    pub_result.publish(result.model_dump_json())

    # エッジ点, 中心軸の点をPublish
    publish_points = np.empty((0, 3))
    publish_colors = np.empty((0, 3))

    for i in range(len(distanceSets)):
        edge_start_point = distanceSets[i].line_segment_points[0]
        edge_end_point = distanceSets[i].line_segment_points[1]
        points_generated = gen_points.segment_to_points(np.asarray(edge_start_point), np.asarray(edge_end_point))
        publish_points = np.concatenate([publish_points, points_generated], axis=0)
        colors = np.zeros((len(points_generated), 3))
        colors[:] = cfg.RGB_TABLE[i]
        publish_colors = np.concatenate([publish_colors, colors])

    points_generated = gen_points.segment_to_points(center - normal * 100, center + normal * 100)
    publish_points = np.concatenate([publish_points, points_generated], axis=0)
    colors = np.zeros((len(points_generated), 3))
    colors[:] = [1, 1, 1]
    publish_colors = np.concatenate([publish_colors, colors])
    publish_pcd = o3d.geometry.PointCloud()
    publish_pcd.points = o3d.utility.Vector3dVector(publish_points)
    publish_pcd.colors = o3d.utility.Vector3dVector(publish_colors)
    # pub_pointcloud.publish(pc2.numpy_to_pc2(publish_points))

    # 画像撮影
    # 画像データ生成を開始
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    outpath = visualize.capture_image(
      [pcd, publish_pcd],
      "/root/src/out",
      "overview.png",
      cam_front=config.Config.CAM_FRONT,
      cam_lookat=result.center,
      cam_up=[0, 0, 1],
      cam_zoom=config.Config.CAM_ZOOM
    )

    # 画像ファイルを読み込んで、ROSMessageに変換してPublish
    with open(outpath, "rb") as f:
        img = cv2.imread(outpath)
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        pub_image.publish(img_msg)


@eel.expose
def read_config():
    json = cfg.to_json()
    return json

@eel.expose
def update_config(json_string: str):
    cfg.update(json_string)
    return