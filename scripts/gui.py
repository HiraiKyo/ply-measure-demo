import os
import sys
import threading
from PyQt6.QtWidgets import QMainWindow, QScrollArea, QApplication, QLabel, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit, QSpacerItem, QSizePolicy
from PyQt6.QtGui import QFont, QTextOption, QFontMetrics, QPixmap
from PyQt6.QtCore import Qt

from utils import config
import datastore as ds
import actions
from python_app_utils.log import Logger

FONT_SIZE = 16

logger = Logger()
datastore = ds.DataStore()
cfg = config.Config()

def open_gui():
  """
  Open Viewer by PyQt6
  """
  qAp = QApplication(sys.argv)
  viewer = MainWindow()
  viewer.show()
  qAp.exec()

  return

class MainWindow(QMainWindow):
  def __init__(self):
    super().__init__()
    self.setWindowTitle("Saisun3D Viewer")
    self.setGeometry(0, 0, 360, 1080)

    self.setup_japanese_font()

    main_widget = QWidget()
    self.setCentralWidget(main_widget)
    v = QVBoxLayout()
    main_widget.setLayout(v)

    v.addWidget(ActionField())
    h = QHBoxLayout()
    v.addLayout(h)

    # ValuesFieldをScrollableにする
    scroll_area = QScrollArea()
    h.addWidget(scroll_area)
    scroll_area.setWidgetResizable(True)
    scroll_area.setWidget(ValuesField())
    # h.addWidget(SnapshotField())
    v.addWidget(LogField())

    return None

  def setup_japanese_font(self):
    # TODO: 日本語フォント対応
    japanese_font = QFont("Yu Gothic UI")
    QApplication.setFont(japanese_font)

class ActionField(QWidget):
  def __init__(self):
    super().__init__()
    # ボタンを配置
    hbox = QHBoxLayout()
    self.setLayout(hbox)

    buttonAuto = QPushButton("Auto", self)
    hbox.addWidget(buttonAuto)
    labelAuto = QLabel("Auto Mode: ")
    hbox.addWidget(labelAuto)
    labelStatusAuto = QLabel("OFF")
    hbox.addWidget(labelStatusAuto)
    buttonSnapshot = QPushButton("Snapshot", self)
    hbox.addWidget(buttonSnapshot)

    spacer = HorizontalSpacer()
    hbox.addItem(spacer)

    buttonClose = QPushButton("Close App", self)
    hbox.addWidget(buttonClose)

    # ボタンのコールバック登録
    buttonAuto.clicked.connect(actions.on_click_auto)
    datastore.auto_mode_changed.connect(lambda value: labelStatusAuto.setText("ON" if value else "OFF"))
    buttonSnapshot.clicked.connect(self.start_snapshot)
    buttonClose.clicked.connect(QApplication.instance().quit)

  def start_snapshot(self):
    th = threading.Thread(target=actions.on_click_snapshot)
    th.setDaemon(True)
    th.start()
class ValuesField(QWidget):
  def __init__(self):
    super().__init__()

    self.vbox = QVBoxLayout()

    self.setLayout(self.vbox)

    self.initUI()
    # 計測ごとにデータ初期化処理が走りリスト長さは0になるため、リスト長変更を検知してデータテーブルを生成する
    datastore.measure_result_changed.connect(self.on_change)
    pass

  def on_change(self, result: dict):
    # データがない場合は何もしない
    if not result:
      return

    # データがある場合はテーブルを生成
    if "distances" in result:
      if len(result["distances"]) == len(self.vbox.children()) + 2:
        return

      # リスト長変更は計測結果の変更を意味するため、テーブルを再生成する
      self.initUI()
      for i, distance in enumerate(result["distances"]):
        v = ValueTableRowBoxLayout(f"Distance {i}", i, color=cfg.RGB_TABLE[i])
        self.vbox.addLayout(v)

  def initUI(self):
    # 子要素をすべて削除する
    clear_layout(self.vbox)

    v1 = ValueFieldBoxLayout(
      "Cylinder Center",
      "center"
    )
    self.vbox.addLayout(v1)

    v2 = ValueFieldBoxLayout(
      "Cylinder Radius",
      "radius"
    )
    self.vbox.addLayout(v2)
    self.vbox.addItem(Spacer())

class ValueFieldBoxLayout(QHBoxLayout):
  def __init__(self, label: str, key: str):
    super().__init__()
    self.key = key

    qlabel = QLabel()
    qlabel.setText(label)
    self.qtext = QTextEdit()
    self.qtext.setReadOnly(True)
    self.qtext.setFont(QFont("Yu Gothic UI", FONT_SIZE))
    self.qtext.setMaximumHeight(QFontMetrics(self.qtext.font()).lineSpacing() + 10)
    self.qtext.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    self.qtext.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    self.qtext.setWordWrapMode(QTextOption.WrapMode.NoWrap)
    self.qtext.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
    self.qtext.setSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.Fixed)

    self.addWidget(qlabel)
    self.addWidget(self.qtext)
    self.addItem(HorizontalSpacer())

    self.on_change(datastore.measure_result.model_dump())
    datastore.measure_result_changed.connect(self.on_change)

  def on_change(self, result: dict):
    if self.key not in result:
      self.qtext.setText("N/A")

    value = result[self.key]
    if isinstance(value, list):
      updatedText = ", ".join([f"{d:.2f}" for d in value])
    else:
      updatedText = f"{value:.2f}"

    # 値が等しい場合には更新を行わない
    if updatedText == self.qtext.toPlainText():
      return

    self.qtext.setText(updatedText)

class ValueTableRowBoxLayout(QHBoxLayout):
  def __init__(self, label, index: int, color=[1, 1, 1]):
    super().__init__()

    self.index = index

    qlabel = QLabel()
    qlabel.setText(label)
    self.addWidget(qlabel)
    self.qtext = QTextEdit()
    self.qtext.setReadOnly(True)
    self.qtext.setFont(QFont("Yu Gothic UI", FONT_SIZE))
    self.qtext.setMaximumHeight(QFontMetrics(self.qtext.font()).lineSpacing() + 10)
    self.qtext.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    self.qtext.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    self.qtext.setWordWrapMode(QTextOption.WrapMode.NoWrap)
    self.qtext.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
    self.qtext.setSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.Fixed)
    self.qtext.setStyleSheet(f"background-color: rgb({color[0] * 255}, {color[1] * 255}, {color[2] * 255});")
    self.addWidget(self.qtext)

    # self.qimg = QLabel()
    # self.qimg.setAlignment(Qt.AlignmentFlag.AlignCenter)
    # self.qimg.setFixedSize(320, 240)
    # self.addWidget(self.qimg)
    # self.addItem(HorizontalSpacer())

    self.on_change(datastore.measure_result.model_dump())
    datastore.measure_result_changed.connect(self.on_change)

  def on_change(self, result: dict):
    currentText = self.qtext.toPlainText()
    # currentImagePath = None
    # try:
    #   currentImagePath = self.qimg.pixmap().toImage().text()
    # except Exception:
    #   currentImagePath = None

    if "distances" not in result:
      return

    distance = result["distances"][self.index]
    if not distance:
      return

    updatedText = f"{distance['distance']:.2f}"
    # 変更があった場合にのみ更新
    if updatedText != currentText:
      self.qtext.setText(updatedText)

    # # 画像パスが更新されている場合にのみ更新
    # updatedImagePath = distance["image_path"]
    # if updatedImagePath != currentImagePath:
    #   if updatedImagePath is None:
    #     self.qimg.clear()
    #   else:
    #     pixmap = QPixmap(f"{updatedImagePath}")
    #     scaled_pixmap = pixmap.scaled(
    #       self.qimg.size(),
    #       Qt.AspectRatioMode.KeepAspectRatio,
    #       Qt.TransformationMode.SmoothTransformation
    #     )
    #     self.qimg.setPixmap(scaled_pixmap)

class SnapshotField(QWidget):
  def __init__(self):
    super().__init__()
    self.setGeometry(0, 0, 640, 480)
    layout = QVBoxLayout()
    self.setLayout(layout)

    self.label = QLabel(self)
    self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    self.label.setFixedSize(640, 480)
    layout.addWidget(self.label)

    layout.addItem(Spacer())
    self.load_image(datastore.image_path)
    datastore.image_path_changed.connect(self.load_image)

  def load_image(self, image_path):
    # 画像パスは str | None で来る
    if image_path is None:
        self.label.clear()
        return

    # 画像パスが更新されない場合は描画も更新しない
    if image_path == self.label.pixmap().toImage().text():
        return

    if not os.path.exists(image_path):
      print(f"Error: Image file not found at {image_path}")
      return
    pixmap = QPixmap(f"{image_path}")
    if not pixmap.isNull():
      scaled_pixmap = pixmap.scaled(
        self.label.size(),
        Qt.AspectRatioMode.KeepAspectRatio,
        Qt.TransformationMode.SmoothTransformation
      )
      self.label.setPixmap(scaled_pixmap)
    else:
      self.label.setText("File is missing or failed to load image.")

class LogField(QWidget):
  def __init__(self):
    super().__init__()
    self.setStyleSheet("background-color: #ff00ff;")
    pass


# GUI Utils
class Spacer(QSpacerItem):
    def __init__(self):
        super().__init__(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        pass

class HorizontalSpacer(QSpacerItem):
    def __init__(self):
        super().__init__(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        pass

class ImagePopup(QLabel):
  def __init__(self, pixmap):
    super().__init__()
    self.setPixmap(pixmap)
    self.setWindowFlags(Qt.WindowType.Popup | Qt.WindowType.WindowStaysOnTopHint)
    self.setAlignment(Qt.AlignmentFlag.AlignCenter)
    self.setScaledContents(True)

    # スクロールエリアを作成
    scroll_area = QScrollArea(self)
    scroll_area.setWidget(self)
    scroll_area.setWidgetResizable(True)

  def mouseReleaseEvent(self, event):
    self.close()

def clear_layout(layout):
  while layout.count():
    child = layout.takeAt(0)
    if child.widget():
      child.widget().deleteLater()
    elif child.layout():
      clear_layout(child.layout())
