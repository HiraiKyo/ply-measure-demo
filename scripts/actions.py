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
  try:
    # 開始前処理
    dataStore.start_run()

    # PhoXiControlでスナップショットを撮影
    pcd = snapshot.take_snapshot()

    # ply-processor-basicsを利用して円柱中心とエッジを検出
    center, radius, normal, plane_indices, line_segment_points, distances, mil_line_segment_points, mil_distances = measure.measure(np.asarray(pcd.points))

    # Datastore層に結果を保存(->FlexパターンでGUI更新)
    distanceSets = [ds.DistanceSet(
      distance=d,
      group=0,
      line_segment_points=(line_segment_points[0][i], line_segment_points[1][i]),
      image_path=None
    ) for i, d in enumerate(distances)]

    result = ds.MeasureResult(
      center=center,
      radius=radius,
      normal=normal,
      distances=distanceSets,
      plane_indices=plane_indices,
    )
    dataStore.update_measure_result(result)

    # # 画像データ生成を開始
    # pcd.paint_uniform_color([0.5, 0.5, 0.5])
    # disk = visualize.generate_disk_mesh(dataStore.measure_result.center, dataStore.measure_result.radius, dataStore.measure_result.normal)
    # line_set = visualize.generate_edge_line(np.asarray(pcd.points), dataStore.measure_result.plane_indices, dataStore.measure_result.line_segments_indices)
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
    #   line_set = visualize.generate_edge_line(np.asarray(pcd.points), plane_indices, [line_segments_indices[i]])
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
    #     line_segment_indices=result.distances[i].line_segment_indices,
    #     image_path=outpath
    #   )
    #   result.distances[i] = updated_distanceSet
    #   dataStore.update_measure_result(result)

    return True
  except Exception as e:
    logger.error(f"Failed to take snapshot. {e}")
    return False
  finally:
    # 終了処理
    dataStore.finish_run()

