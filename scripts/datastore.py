from python_app_utils.singleton import Singleton
from pydantic import BaseModel
from typing import List
import time
import os
import io
import csv
from python_app_utils.log import Logger
class MeasureResult(BaseModel):
  center: List[float]
  radius: float
  normal: List[float]
  distances: List[float]
  plane_indices: List[int]
  line_segments_indices: List[List[int]]

logger = Logger()

class DataStore(Singleton):
  running_state: bool = False
  loop_mode: bool = False

  measure_result: MeasureResult
  outdir: str

  def __init__(self):
    self.measure_result = MeasureResult(
      center=[0, 0, 1],
      radius=0.0,
      normal=[0, 0, 1],
      distances=[0, 0, 0],
      plane_indices=[0, 1, 2],
      line_segments_indices=[[0, 1],[1, 2]]
    )
    pass

  def update_measure_result(self, result: MeasureResult):
    self.measure_result = result
    # PydanticからCSV保存
    fields = list(self.measure_result.model_fields.keys())

    output = io.StringIO()
    writer = csv.DictWriter(output, fieldnames=fields)
    writer.writeheader()
    writer.writerow(result.model_dump())

    logger.info("Update measure result")
    return

  def update_image(self, image_path: str):
    self.image_path = image_path
    return
  

  def start_run(self):
    self.running_state = True
    # 開始時に出力先ディレクトリをtimestampから作成
    timestamp = time.strftime("%Y%m%d%H%M%S", time.localtime())
    self.outdir = f"out/{timestamp}"
    os.makedirs(self.outdir, exist_ok=True)

    logger.info(f"Start run: {self.outdir}")

  def finish_run(self):
    self.running_state = False
    logger.export(self.outdir)