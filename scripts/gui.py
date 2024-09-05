import sys
from PyQt6.QtWidgets import QMainWindow, QApplication, QLabel, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit, QSpacerItem, QSizePolicy
from PyQt6.QtGui import QFont, QTextOption, QFontMetrics
from PyQt6.QtCore import Qt

from datastore import DataStore
from actions import on_click_auto, on_click_snapshot


FONT_SIZE = 16

def open_gui():
  """
  Open Viewer by PyQt6
  """
  # データストア初期化
  DataStore()

  qAp = QApplication(sys.argv)
  viewer = MainWindow()
  viewer.show()
  qAp.exec()

  return None

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

    # TODO: Reactive UI
    datastore = DataStore()

    vbox = QVBoxLayout()
    self.setLayout(vbox)

    v1 = ValueFieldBoxLayout(
      "Cylinder Center",
      ", ".join([f"{d:.2f}" for d in datastore.measure_result.center])
    )
    vbox.addLayout(v1)

    v2 = ValueFieldBoxLayout(
      "Cylinder Radius",
      f"{datastore.measure_result.radius:.2f}"
    )
    vbox.addLayout(v2)

    v3 = ValueFieldBoxLayout(
      "Distances from edges",
      ", ".join([f"{d:.2f}" for d in datastore.measure_result.distances])
    )
    vbox.addLayout(v3)


    vbox.addItem(Spacer())
    pass

class ValueFieldBoxLayout(QHBoxLayout):
  def __init__(self, label, value):
    super().__init__()
    qlabel = QLabel()
    qlabel.setText(label)

    qtext = QTextEdit()
    qtext.setReadOnly(True)
    qtext.setText(value)
    qtext.setFont(QFont("Yu Gothic UI", FONT_SIZE))
    qtext.setMaximumHeight(QFontMetrics(qtext.font()).lineSpacing() + 10)
    qtext.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    qtext.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    qtext.setWordWrapMode(QTextOption.WrapMode.NoWrap)
    qtext.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
    qtext.setSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.Fixed)

    self.addWidget(qlabel)
    self.addWidget(qtext)
    self.addItem(HorizontalSpacer())

class SnapshotField(QWidget):
  def __init__(self):
    super().__init__()
    self.setStyleSheet("background-color: #ffff00;")
    pass

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