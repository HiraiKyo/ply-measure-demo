from python_app_utils.singleton import Singleton
from pydantic import BaseModel
from typing import List

class MeasureResult(BaseModel):
  center: List[float]
  radius: float
  distances: List[float]
  edges: List[List[float]]

class DataStore(Singleton):
  running_state: bool = False

  measure_result: MeasureResult
  outdir: str

  def __init__(self):
    self.measure_result = MeasureResult(
      center=[0, 0, 1],
      radius=0.0,
      distances=[0, 0, 0],
      edges=[[0, 0], [0, 0]]
    )
    pass

  def update_measure_result(self, result: MeasureResult):
    self.measure_result = result
    return None

  def write_out(self):
    # TODO: ファイル出力
    # 出力ディレクトリ名を日付から生成

    # MeasureResultは1行のCSVとしてresult.csvに出力

    # 画像ファイルをout.pngに出力

    # ログファイルをlog.txtに出力

    self.finish_run()
    pass

  def start_run(self):
    self.running_state = True

  def finish_run(self):
    self.running_state = False