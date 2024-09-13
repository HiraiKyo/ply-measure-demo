import numpy as np

def segment_to_points(start, end):
  """
  始点と終点から線分を構成する点を生成する
  """
  # 線分の長さを計算
  length = np.linalg.norm(end - start)
  normalized = (end - start) / length
  interval = 1.0
  points = []
  for i in range(int(length / interval)):
    points.append(start + normalized * interval * i)
  points.append(end)
  return np.array(points)