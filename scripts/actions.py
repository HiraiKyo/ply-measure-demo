from models.snapshot import take_snapshot
from python_app_utils.log import Logger
from models.measure import measure
import numpy as np

from models.visualize import generate_disk_mesh, generate_edge_line, capture_image
from datastore import DataStore, MeasureResult
from utils.config import Config

dataStore = DataStore()

def on_click_auto():
  # TODO: ループ実装
  pass

def on_click_snapshot():
  logger = Logger()
  try:
    # 開始前処理
    dataStore.start_run()

    # PhoXiControlでスナップショットを撮影
    pcd = take_snapshot()

    # ply-processor-basicsを利用して円柱中心とエッジを検出
    center, radius, normal, distances, plane_indices, line_segments_indices = measure(np.asarray(pcd.points))

    # Datastore層に結果を保存(->FlexパターンでGUI更新)
    result = MeasureResult(
      center=center,
      radius=radius,
      normal=normal,
      distances=distances,
      plane_indices=plane_indices,
      line_segments_indices=line_segments_indices
    )
    dataStore.update_measure_result(result)

    # 画像データ生成を開始
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    disk = generate_disk_mesh(dataStore.measure_result.center, dataStore.measure_result.radius, dataStore.measure_result.normal)
    line_set = generate_edge_line(np.asarray(pcd.points), dataStore.measure_result.plane_indices, dataStore.measure_result.line_segments_indices)
    outpath = capture_image(
      [pcd, disk, line_set],
      dataStore.outdir,
      cam_front=Config.CAM_FRONT,
      cam_lookat=dataStore.measure_result.center,
      cam_up=[0, 0, 1],
      cam_zoom=Config.CAM_ZOOM
    )
    dataStore.update_image(outpath)
  except Exception as e:
    logger.error(f"Failed to take snapshot. {e}")
  finally:
    # 終了処理
    dataStore.finish_run()
