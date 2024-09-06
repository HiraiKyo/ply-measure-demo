from datetime import datetime
from pydantic import BaseModel
from typing import List
import time
import os
import io
import csv
from python_app_utils.log import Logger
from PyQt6.QtCore import pyqtSignal, QObject

from typing import Union

class DistanceSet(BaseModel):
  distance: float
  line_segment_indices: List[int]
  image_path: Union[str, None]

class MeasureResult(BaseModel):
  center: List[float]
  radius: float
  normal: List[float]
  distances: List[DistanceSet]
  plane_indices: List[int]
  line_segments_indices: List[List[int]]

logger = Logger()

class DataStore(QObject):
  outdir: str

  running_state_changed = pyqtSignal(bool)
  @property
  def running_state(self):
    return self._running_state
  @running_state.setter
  def running_state(self, value):
    if self._running_state != value:
      self._running_state = value
      self.running_state_changed.emit(value)

  measure_result_changed = pyqtSignal(dict)
  @property
  def measure_result(self):
    return self._measure_result
  @measure_result.setter
  def measure_result(self, value: MeasureResult):
    self._measure_result = value
    self.measure_result_changed.emit(value.model_dump())

  image_path_changed = pyqtSignal(str)
  @property
  def image_path(self):
    return self._image_path
  @image_path.setter
  def image_path(self, value):
    self._image_path = value
    self.image_path_changed.emit(value)

  # Singletonパターン
  _instance = None
  _initialized = False
  def __new__(cls):
    if cls._instance is None:
      cls._instance = super(DataStore, cls).__new__(cls)
    return cls._instance

  def __init__(self, *args, **kwargs):
    # Singletonパターン記述
    if self._initialized:
      return
    self._initialized = True

    super(DataStore, self).__init__(*args, **kwargs)
    self._running_state = True
    self.initialize_data()
    self._running_state = False
    pass

  def update_measure_result(self, result: MeasureResult):
    # result validation
    assert len(result.distances) == len(result.line_segments_indices)

    self.measure_result = result
    # PydanticからCSV保存
    fields = list(self.measure_result.model_fields.keys())

    output = io.StringIO()
    writer = csv.DictWriter(output, fieldnames=fields)
    writer.writeheader()
    writer.writerow(result.model_dump())
    return

  def update_image(self, image_path: str):
    self.image_path = image_path
    self.image_path_changed.emit(image_path)
    return


  def start_run(self):
    self.running_state = True

    # 計算開始時は明示的にデータストア初期化
    self.initialize_data()

    # 開始時に出力先ディレクトリをtimestampから作成
    timestamp = time.strftime("%Y%m%d%H%M%S", time.localtime())
    self.outdir = f"out/{timestamp}"
    os.makedirs(self.outdir, exist_ok=True)

    logger.info(f"Start run: {self.outdir}")

  def finish_run(self):
    self.running_state = False
    logger.export(self.outdir)

  def initialize_data(self):
    self.outdir = None
    self.measure_result = MeasureResult(
      center=[],
      radius=0.0,
      normal=[],
      distances=[],
      plane_indices=[],
      line_segments_indices=[]
    )
    self.image_path = None
