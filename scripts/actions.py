from models import snapshot
from python_app_utils.log import Logger
import numpy as np
import open3d as o3d
from models import visualize, measure
import datastore as ds
from utils import config, pc2
from sensor_msgs.msg import PointCloud2

dataStore = ds.DataStore()
logger = Logger()

def on_click_auto():
  dataStore.auto_mode = not dataStore.auto_mode
  pass

# 別スレッドで処理を実行する
def on_click_snapshot() -> bool:
  try:
    # 開始前処理
    dataStore.start_run()

    # PhoXiControlでスナップショットを撮影
    pcd = snapshot.take_snapshot()

    # ply-processor-basicsを利用して円柱中心とエッジを検出
    center, radius, normal, distances, plane_indices, line_segments_indices = measure.measure(np.asarray(pcd.points))

    # Datastore層に結果を保存(->FlexパターンでGUI更新)
    distanceSets = [ds.DistanceSet(
      distance=d,
      line_segment_indices=line_segments_indices[i],
      image_path=None
    ) for i, d in enumerate(distances)]

    result = ds.MeasureResult(
      center=center,
      radius=radius,
      normal=normal,
      distances=distanceSets,
      plane_indices=plane_indices,
      line_segments_indices=line_segments_indices
    )
    dataStore.update_measure_result(result)

    # 画像データ生成を開始
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    disk = visualize.generate_disk_mesh(dataStore.measure_result.center, dataStore.measure_result.radius, dataStore.measure_result.normal)
    line_set = visualize.generate_edge_line(np.asarray(pcd.points), dataStore.measure_result.plane_indices, dataStore.measure_result.line_segments_indices)
    outpath = visualize.capture_image(
      [pcd, disk, line_set],
      dataStore.outdir,
      "overview.png",
      cam_front=config.Config.CAM_FRONT,
      cam_lookat=dataStore.measure_result.center,
      cam_up=[0, 0, 1],
      cam_zoom=config.Config.CAM_ZOOM
    )
    dataStore.update_image(outpath)

    # 各距離用の画像生成
    for i in range(len(distances)):
      line_set = visualize.generate_edge_line(np.asarray(pcd.points), plane_indices, [line_segments_indices[i]])
      outpath = visualize.capture_image(
        [disk, line_set],
        dataStore.outdir,
        f"distance_{i}.png",
        cam_front=config.Config.CAM_FRONT,
        cam_lookat=dataStore.measure_result.center,
        cam_up=[0, 0, 1],
        cam_zoom=config.Config.CAM_ZOOM
      )
      updated_distanceSet = ds.DistanceSet(
        distance=result.distances[i].distance,
        line_segment_indices=result.distances[i].line_segment_indices,
        image_path=outpath
      )
      result.distances[i] = updated_distanceSet
      dataStore.update_measure_result(result)

    return True
  except Exception as e:
    logger.error(f"Failed to take snapshot. {e}")
    return False
  finally:
    # 終了処理
    dataStore.finish_run()

def rossub_callback(msg: PointCloud2):
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
    center, radius, normal, distances, plane_indices, line_segments_indices = measure.measure(points)

    # Datastore層に結果を保存(->FlexパターンでGUI更新)
    distanceSets = [ds.DistanceSet(
      distance=d,
      line_segment_indices=line_segments_indices[i],
      image_path=None
    ) for i, d in enumerate(distances)]

    result = ds.MeasureResult(
      center=center,
      radius=radius,
      normal=normal,
      distances=distanceSets,
      plane_indices=plane_indices,
      line_segments_indices=line_segments_indices
    )
    dataStore.update_measure_result(result)

    # 画像データ生成を開始
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    disk = visualize.generate_disk_mesh(dataStore.measure_result.center, dataStore.measure_result.radius, dataStore.measure_result.normal)
    line_set = visualize.generate_edge_line(points, dataStore.measure_result.plane_indices, dataStore.measure_result.line_segments_indices)
    outpath = visualize.capture_image(
      [pcd, disk, line_set],
      dataStore.outdir,
      "overview.png",
      cam_front=config.Config.CAM_FRONT,
      cam_lookat=dataStore.measure_result.center,
      cam_up=[0, 0, 1],
      cam_zoom=config.Config.CAM_ZOOM
    )
    dataStore.update_image(outpath)

    # 各距離用の画像生成
    for i in range(len(distances)):
      line_set = visualize.generate_edge_line(points, plane_indices, [line_segments_indices[i]])
      outpath = visualize.capture_image(
        [disk, line_set],
        dataStore.outdir,
        f"distance_{i}.png",
        cam_front=config.Config.CAM_FRONT,
        cam_lookat=dataStore.measure_result.center,
        cam_up=[0, 0, 1],
        cam_zoom=config.Config.CAM_ZOOM
      )
      updated_distanceSet = ds.DistanceSet(
        distance=result.distances[i].distance,
        line_segment_indices=result.distances[i].line_segment_indices,
        image_path=outpath
      )
      result.distances[i] = updated_distanceSet
      dataStore.update_measure_result(result)

    return
  except Exception as e:
    logger.error(f"Failed to take snapshot. {e}")
    return
  finally:
    # 終了処理
    dataStore.finish_run()