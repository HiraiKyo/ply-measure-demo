"""
PubSub Manager for ROS
"""

from python_app_utils.log import Logger
from python_app_utils.singleton import Singleton
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import rospy
import numpy as np
import open3d as o3d
import traceback
import datastore as ds
from models import measure
from models import gen_points
from utils import config, pc2

dataStore = ds.DataStore()
logger = Logger()
cfg = config.Config()

class ROSNodeManager(Singleton):
  def __init__(self):
    rospy.init_node("ply_measure_demo", anonymous=True)

    # Publishers
    self.pub_result = rospy.Publisher(cfg.ROS_PUB_TOPIC_RESULT, String, queue_size=1)
    self.pub_pointcloud = rospy.Publisher(cfg.ROS_PUB_TOPIC_POINTCLOUD, PointCloud2, queue_size=1)

    # Subscribers
    self.sub = rospy.Subscriber(cfg.ROS_SUB_TOPIC, PointCloud2, self.rossub_callback)

  def publish_result(self, msg):
    logger.info("Publishing Result")
    self.pub_result.publish(msg)

  def publish_pointcloud(self, msg):
    logger.info("Publishing PointCloud")
    self.pub_pointcloud.publish(msg)

  def rossub_callback(self, msg: PointCloud2):
    """
    ROS Subscriber Callback
    """
    try:
      # 開始前処理
      dataStore.start_run()

      # auto_modeか確認
      if not dataStore.auto_mode:
        return

      # PointCloud2 msgを変換
      points = pc2.pc2_to_numpy(msg)
      pcd = o3d.geometry.PointCloud()
      pcd.points = o3d.utility.Vector3dVector(points)

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

      # # 画像データ生成を開始
      # pcd.paint_uniform_color([0.5, 0.5, 0.5])
      # disk = visualize.generate_disk_mesh(dataStore.measure_result.center, dataStore.measure_result.radius, dataStore.measure_result.normal)
      # line_set = visualize.generate_edge_line(points, dataStore.measure_result.plane_indices, dataStore.measure_result.line_segments_indices)
      # outpath = visualize.capture_image(
      #   [pcd, disk, line_set],
      #   dataStore.outdir,
      #   "overview.png",
      #   cam_front=config.Config.CAM_FRONT,
      #   cam_lookat=dataStore.measure_result.center,
      #   cam_up=[0, 0, 1],
      #   cam_zoom=config.Config.CAM_ZOOM
      # )
      # dataStore.update_image(outpath)

      # # 各距離用の画像生成
      # for i in range(len(distances)):
      #   line_set = visualize.generate_edge_line(points, plane_indices, [line_segments_indices[i]])
      #   outpath = visualize.capture_image(
      #     [disk, line_set],
      #     dataStore.outdir,
      #     f"distance_{i}.png",
      #     cam_front=config.Config.CAM_FRONT,
      #     cam_lookat=dataStore.measure_result.center,
      #     cam_up=[0, 0, 1],
      #     cam_zoom=config.Config.CAM_ZOOM
      #   )
      #   updated_distanceSet = ds.DistanceSet(
      #     distance=result.distances[i].distance,
      #     line_segment_points=result.distances[i].line_segment_points,
      #     image_path=outpath
      #   )
      #   result.distances[i] = updated_distanceSet
      #   dataStore.update_measure_result(result)

    except Exception as e:
      logger.error(f"Failed to take snapshot. {e}")
      print(traceback.format_tb(e.__traceback__))
    finally:
      # 終了処理
      dataStore.finish_run()
