import os
import sys
from PyQt6.QtWidgets import QMainWindow, QApplication, QLabel, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit, QSpacerItem, QSizePolicy
from PyQt6.QtGui import QFont, QTextOption, QFontMetrics, QPixmap
from PyQt6.QtCore import Qt

from datastore import DataStore, MeasureResult
from actions import on_click_auto, on_click_snapshot
from python_app_utils.log import Logger

FONT_SIZE = 16

logger = Logger()
datastore = DataStore()

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
    self.setGeometry(0, 0, 1920, 1080)

    self.setup_japanese_font()

    main_widget = QWidget()
    self.setCentralWidget(main_widget)
    v = QVBoxLayout()
    main_widget.setLayout(v)

    v.addWidget(ActionField())
    h = QHBoxLayout()
    v.addLayout(h)
    h.addWidget(ValuesField())
    h.addWidget(SnapshotField())
    v.addWidget(LogField())

    spacer = Spacer()
    v.addItem(spacer)

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
    buttonSnapshot = QPushButton("Snapshot", self)
    hbox.addWidget(buttonSnapshot)

    spacer = HorizontalSpacer()
    hbox.addItem(spacer)

    buttonClose = QPushButton("Close App", self)
    hbox.addWidget(buttonClose)

    # ボタンのコールバック登録
    buttonAuto.clicked.connect(on_click_auto)
    buttonSnapshot.clicked.connect(on_click_snapshot)
    buttonClose.clicked.connect(QApplication.instance().quit)

class ValuesField(QWidget):
  def __init__(self):
    super().__init__()

    vbox = QVBoxLayout()
    self.setLayout(vbox)

    v1 = ValueFieldBoxLayout(
      "Cylinder Center",
      "center"
    )
    vbox.addLayout(v1)

    v2 = ValueFieldBoxLayout(
      "Cylinder Radius",
      "radius"
    )
    vbox.addLayout(v2)

    v3 = ValueFieldBoxLayout(
      "Distances from edges",
      "distances"
    )
    vbox.addLayout(v3)


    vbox.addItem(Spacer())
    pass

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

    datastore.measure_result_changed.connect(self.on_change)

  def on_change(self, result: dict):
    if self.key not in result:
      self.qtext.setText("N/A")

    value = result[self.key]
    if isinstance(value, list):
      self.qtext.setText(", ".join([f"{d:.2f}" for d in value]))
    else:
      self.qtext.setText(f"{value:.2f}")
  
class SnapshotField(QWidget):
  def __init__(self):
    super().__init__()
    self.setGeometry(0, 0, 640, 480)
    # 画像を表示
    if datastore.image_path is None:
        pass
    layout = QVBoxLayout()
    self.setLayout(layout)

    self.label = QLabel(self)
    self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    self.label.setFixedSize(640, 480)
    layout.addWidget(self.label)

    datastore.image_path_changed.connect(self.load_image)

  def load_image(self, image_path):
    print(image_path)
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
      print(f"Pixmap set to label. Size: {scaled_pixmap.size()}")
      # self.label.repaint()
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