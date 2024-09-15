from models import snapshot
from python_app_utils.log import Logger
import numpy as np
import open3d as o3d
from models import visualize, measure
import datastore as ds
from models import gen_points
from utils import config, pc2
from sensor_msgs.msg import PointCloud2
import rosnode

dataStore = ds.DataStore()
logger = Logger()
def on_click_auto():
      dataStore.auto_mode = not dataStore.auto_mode
      pass

# 別スレッドで処理を実行する
def on_click_snapshot() -> bool:
  # 開始前処理
      dataStore.start_run()

      pcd = snapshot.take_snapshot()
      points = np.asarray(pcd.points)

      # ply-processor-basicsを利用して円柱中心とエッジを検出
      center, radius, normal, plane_indices, line_segment_points, distances, mil_line_segment_points, mil_distances = measure.measure(points)

      # Datastore層に結果を保存(->FlexパターンでGUI更新)
      distanceSets = [ds.DistanceSet(
        distance=d,
        group=0,
        line_segment_points=(line_segment_points[0][i], line_segment_points[1][i]),
        image_path=None
      ) for i, d in enumerate(distances)]
      distanceSets += [ds.DistanceSet(
        distance=d,
        group=1,
        line_segment_points=(mil_line_segment_points[0][i], mil_line_segment_points[1][i]),
        image_path=None
      ) for i, d in enumerate(mil_distances)]

      result = ds.MeasureResult(
        center=center,
        radius=radius,
        normal=normal,
        distances=distanceSets,
        plane_indices=plane_indices,
      )
      dataStore.update_measure_result(result)

      # ROSにトピックをPublish
      self.publish_result(result.model_dump_json())

      publish_points = np.empty((0, 3))
      publish_colors = np.empty((0, 3))
      # エッジ点をPublish
      for i in range(len(distanceSets)):
        edge_start_point = distanceSets[i].line_segment_points[0]
        edge_end_point = distanceSets[i].line_segment_points[1]
        points_generated = gen_points.segment_to_points(edge_start_point, edge_end_point)
        publish_points = np.concatenate([publish_points, points_generated], axis=0)
        colors = np.zeros((len(points_generated), 3))
        colors[:] = cfg.RGB_TABLE[i]
        publish_colors = np.concatenate([publish_colors, colors])

      # 中心軸の点をPublish
      points_generated = gen_points.segment_to_points(center - normal * 100, center + normal * 100)
      publish_points = np.concatenate([publish_points, points_generated], axis=0)
      colors = np.zeros((len(points_generated), 3))
      colors[:] = [1, 1, 1]
      publish_colors = np.concatenate([publish_colors, colors])
      publish_pcd = o3d.geometry.PointCloud()
      publish_pcd.points = o3d.utility.Vector3dVector(publish_points)
      publish_pcd.colors = o3d.utility.Vector3dVector(publish_colors)
      self.publish_pointcloud(pc2.to_msg(publish_pcd, frame_id="camera"))

