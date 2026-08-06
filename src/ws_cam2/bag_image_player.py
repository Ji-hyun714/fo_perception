#!/usr/bin/env python3
"""
ROS2 bag camera-image PLAYER (GUI)

기능
  - 동영상 플레이어 형태: 재생/일시정지, 진행 바(seek), 배속 조절
  - 1프레임 단위 이전/다음 이동, 슬라이더, 프레임 번호 직접 입력
  - 표시 정보
      · bag 이름 / bag 전체 길이 / 토픽 / 총 프레임
      · 현재 프레임 번호 / 총 프레임
      · 전체 시간에서 경과 시간 (ms 단위)
      · BAG 기록 시각 (bag clock, 절대 ns + 경과)
      · 센서 타임스탬프 (메시지 header.stamp, 절대 ns + 경과)  ← 둘 다 표시
  - 현재 프레임 저장(S), 범위 일괄 저장
실행
  python bag_image_player.py                              # 폴더 선택창
  python bag_image_player.py /path/to/bag                 # 경로 지정
  python bag_image_player.py /path/to/bag -t /cam/image_raw -o saved --ext jpg
의존성
  ROS2 환경(source 후), rosbag2_py, cv_bridge, sensor_msgs
  pip install opencv-python numpy PyQt5
"""

import os
import sys
import argparse

import numpy as np
import cv2

from PyQt5 import QtCore, QtGui, QtWidgets

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")


# --------------------------------------------------------------------------- #
#  bag 읽기 유틸
# --------------------------------------------------------------------------- #
def detect_storage_id(bag_path):
    if os.path.isdir(bag_path):
        names = os.listdir(bag_path)
        if any(n.endswith(".mcap") for n in names):
            return "mcap"
        if any(n.endswith(".db3") for n in names):
            return "sqlite3"
    return "sqlite3"


def open_reader(bag_path, storage_id):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    return reader


def list_image_topics(bag_path, storage_id):
    reader = open_reader(bag_path, storage_id)
    return [(t.name, t.type) for t in reader.get_all_topics_and_types()
            if t.type in IMAGE_TYPES]


def header_stamp_ns(msg):
    """sensor_msgs Image/CompressedImage 의 header.stamp 를 ns 로."""
    try:
        s = msg.header.stamp
        return int(s.sec) * 1_000_000_000 + int(s.nanosec)
    except Exception:
        return None


class FrameIndex:
    """
    선택 토픽의 프레임 인덱스.
    스캔 시 bag clock(ns)·센서 stamp(ns)·직렬화 데이터를 보관하고,
    이미지 디코딩은 요청 시점에 수행 + LRU 캐시.
    """
    def __init__(self, bag_path, storage_id, topic, topic_type, cache_size=96):
        self.bag_path = bag_path
        self.storage_id = storage_id
        self.topic = topic
        self.is_compressed = topic_type == "sensor_msgs/msg/CompressedImage"
        self.bridge = CvBridge()
        self.bag_ts = []        # bag 기록 시각 (ns)
        self.sensor_ts = []     # 센서 header.stamp (ns), 없으면 None
        self._raw = []
        self._cache = {}
        self._cache_order = []
        self._cache_size = cache_size
        self._scan()

    def _scan(self):
        reader = open_reader(self.bag_path, self.storage_id)
        cls = CompressedImage if self.is_compressed else Image
        while reader.has_next():
            tp, data, t_ns = reader.read_next()
            if tp != self.topic:
                continue
            self.bag_ts.append(t_ns)
            # header.stamp 만 빠르게 보려고 전체 역직렬화 (이미지 포함이지만
            # 캐시 안 함 -> 메모리엔 직렬화 bytes 만 남김)
            try:
                msg = deserialize_message(data, cls)
                self.sensor_ts.append(header_stamp_ns(msg))
            except Exception:
                self.sensor_ts.append(None)
            self._raw.append(data)

    def __len__(self):
        return len(self.bag_ts)

    # --- bag clock 기준 ---
    @property
    def bag_start(self):
        return self.bag_ts[0] if self.bag_ts else 0

    @property
    def bag_end(self):
        return self.bag_ts[-1] if self.bag_ts else 0

    @property
    def duration_ns(self):
        return self.bag_end - self.bag_start

    def bag_elapsed_ns(self, idx):
        return self.bag_ts[idx] - self.bag_start

    # --- 센서 stamp 기준 ---
    @property
    def sensor_start(self):
        for v in self.sensor_ts:
            if v is not None:
                return v
        return None

    def sensor_elapsed_ns(self, idx):
        s0 = self.sensor_start
        v = self.sensor_ts[idx]
        if s0 is None or v is None:
            return None
        return v - s0

    def get_image(self, idx):
        if idx in self._cache:
            return self._cache[idx]
        data = self._raw[idx]
        if self.is_compressed:
            msg = deserialize_message(data, CompressedImage)
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        else:
            msg = deserialize_message(data, Image)
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._cache_put(idx, img)
        return img

    def _cache_put(self, idx, img):
        self._cache[idx] = img
        self._cache_order.append(idx)
        if len(self._cache_order) > self._cache_size:
            old = self._cache_order.pop(0)
            self._cache.pop(old, None)


# --------------------------------------------------------------------------- #
#  시간 포맷
# --------------------------------------------------------------------------- #
def fmt_ms(ns):
    """경과 ns -> ('MM:SS.mmm', 초(float)).  ns가 None이면 ('--', None)."""
    if ns is None:
        return "--:--.---", None
    total_ms = ns / 1e6
    minutes = int(total_ms // 60000)
    seconds = int((total_ms % 60000) // 1000)
    ms = int(total_ms % 1000)
    return f"{minutes:02d}:{seconds:02d}.{ms:03d}", total_ms / 1000.0


def fmt_duration(ns):
    s = ns / 1e9
    minutes = int(s // 60)
    seconds = s % 60
    return f"{minutes:02d}:{seconds:06.3f}"


def fmt_abs_ns(ns):
    """절대 ns -> 'sec.nnnnnnnnn' (사람이 읽기 쉽게 초.나노)."""
    if ns is None:
        return "N/A"
    sec = ns // 1_000_000_000
    nano = ns % 1_000_000_000
    return f"{sec}.{nano:09d}"


# --------------------------------------------------------------------------- #
#  토픽 선택 다이얼로그
# --------------------------------------------------------------------------- #
class TopicDialog(QtWidgets.QDialog):
    def __init__(self, topics, parent=None):
        super().__init__(parent)
        self.setWindowTitle("이미지 토픽 선택")
        self.selected = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("재생할 이미지 토픽을 선택하세요:"))
        self.listw = QtWidgets.QListWidget()
        for name, ttype in topics:
            self.listw.addItem(f"{name}    [{ttype.split('/')[-1]}]")
        self.listw.setCurrentRow(0)
        self.listw.itemDoubleClicked.connect(self.accept)
        layout.addWidget(self.listw)
        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)
        self._topics = topics
        self.resize(460, 300)

    def accept(self):
        row = self.listw.currentRow()
        if row >= 0:
            self.selected = self._topics[row][0]
        super().accept()


# --------------------------------------------------------------------------- #
#  메인 플레이어 윈도우
# --------------------------------------------------------------------------- #
class PlayerWindow(QtWidgets.QMainWindow):
    SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]

    def __init__(self, index: FrameIndex, bag_name, output_dir, ext):
        super().__init__()
        self.index = index
        self.bag_name = bag_name
        self.output_dir = output_dir
        self.ext = ext
        self.idx = 0
        self.saved = set()
        self.playing = False
        self.speed = 1.0
        os.makedirs(self.output_dir, exist_ok=True)

        # 프레임 간 실제 간격(bag clock 기준)으로 재생 타이밍 계산
        self._build_intervals()

        self.timer = QtCore.QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._advance)

        self.setWindowTitle(f"ROS2 Bag Image Player — {bag_name}")
        self.resize(1180, 880)
        self._build_ui()
        self._update_view()

    def _build_intervals(self):
        ts = self.index.bag_ts
        self.intervals_ms = []
        for i in range(len(ts)):
            if i + 1 < len(ts):
                self.intervals_ms.append(max(1, (ts[i + 1] - ts[i]) / 1e6))
            else:
                self.intervals_ms.append(self.intervals_ms[-1] if self.intervals_ms else 33.0)

    # ---- UI ---------------------------------------------------------------- #
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # 상단 bag 정보
        info_box = QtWidgets.QGroupBox("Bag 정보")
        g = QtWidgets.QGridLayout(info_box)
        self.lbl_bag = QtWidgets.QLabel(self.bag_name)
        self.lbl_duration = QtWidgets.QLabel(fmt_duration(self.index.duration_ns))
        self.lbl_topic = QtWidgets.QLabel(self.index.topic)
        self.lbl_frames = QtWidgets.QLabel(str(len(self.index)))
        g.addWidget(QtWidgets.QLabel("<b>Bag 이름</b>"), 0, 0)
        g.addWidget(self.lbl_bag, 0, 1)
        g.addWidget(QtWidgets.QLabel("<b>전체 길이</b>"), 0, 2)
        g.addWidget(self.lbl_duration, 0, 3)
        g.addWidget(QtWidgets.QLabel("<b>토픽</b>"), 1, 0)
        g.addWidget(self.lbl_topic, 1, 1)
        g.addWidget(QtWidgets.QLabel("<b>총 프레임</b>"), 1, 2)
        g.addWidget(self.lbl_frames, 1, 3)
        root.addWidget(info_box)

        # 이미지 표시
        self.image_label = QtWidgets.QLabel("no image")
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.image_label.setMinimumHeight(440)
        self.image_label.setStyleSheet("background:#101010; color:#888;")
        root.addWidget(self.image_label, stretch=1)

        # 진행 바 + 시간 라벨 (좌: 경과, 우: 전체)
        seek_row = QtWidgets.QHBoxLayout()
        self.lbl_elapsed = QtWidgets.QLabel("00:00.000")
        self.lbl_total = QtWidgets.QLabel(fmt_duration(self.index.duration_ns))
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(max(0, len(self.index) - 1))
        self.slider.sliderPressed.connect(self._pause)
        self.slider.valueChanged.connect(self._on_slider)
        seek_row.addWidget(self.lbl_elapsed)
        seek_row.addWidget(self.slider, stretch=1)
        seek_row.addWidget(self.lbl_total)
        root.addLayout(seek_row)

        # 재생 컨트롤 줄
        ctrl = QtWidgets.QHBoxLayout()
        self.btn_first = QtWidgets.QPushButton("⏮")
        self.btn_prev = QtWidgets.QPushButton("◀ 이전")
        self.btn_play = QtWidgets.QPushButton("▶ 재생")
        self.btn_next = QtWidgets.QPushButton("다음 ▶")
        self.btn_last = QtWidgets.QPushButton("⏭")
        self.btn_first.clicked.connect(lambda: self._goto(0))
        self.btn_prev.clicked.connect(lambda: self._step(-1))
        self.btn_play.clicked.connect(self._toggle_play)
        self.btn_next.clicked.connect(lambda: self._step(+1))
        self.btn_last.clicked.connect(lambda: self._goto(len(self.index) - 1))
        for w in (self.btn_first, self.btn_prev, self.btn_play, self.btn_next, self.btn_last):
            ctrl.addWidget(w)

        ctrl.addSpacing(16)
        ctrl.addWidget(QtWidgets.QLabel("배속"))
        self.cmb_speed = QtWidgets.QComboBox()
        for s in self.SPEEDS:
            self.cmb_speed.addItem(f"{s}x", s)
        self.cmb_speed.setCurrentIndex(self.SPEEDS.index(1.0))
        self.cmb_speed.currentIndexChanged.connect(self._on_speed)
        ctrl.addWidget(self.cmb_speed)

        self.chk_loop = QtWidgets.QCheckBox("반복")
        ctrl.addWidget(self.chk_loop)

        ctrl.addStretch(1)
        ctrl.addWidget(QtWidgets.QLabel("프레임:"))
        self.spin = QtWidgets.QSpinBox()
        self.spin.setMinimum(1)
        self.spin.setMaximum(max(1, len(self.index)))
        ctrl.addWidget(self.spin)
        self.btn_go = QtWidgets.QPushButton("이동")
        self.btn_go.clicked.connect(lambda: self._goto(self.spin.value() - 1))
        ctrl.addWidget(self.btn_go)
        root.addLayout(ctrl)

        # 타임스탬프 패널 (bag clock + 센서 stamp 둘 다)
        ts_box = QtWidgets.QGroupBox("타임스탬프")
        tg = QtWidgets.QGridLayout(ts_box)
        self.lbl_frame_no = QtWidgets.QLabel()
        self.lbl_overall = QtWidgets.QLabel()
        self.lbl_bag_abs = QtWidgets.QLabel()
        self.lbl_bag_el = QtWidgets.QLabel()
        self.lbl_sensor_abs = QtWidgets.QLabel()
        self.lbl_sensor_el = QtWidgets.QLabel()
        mono = QtGui.QFont("monospace")
        for w in (self.lbl_bag_abs, self.lbl_bag_el,
                  self.lbl_sensor_abs, self.lbl_sensor_el, self.lbl_overall):
            w.setFont(mono)
        tg.addWidget(QtWidgets.QLabel("<b>프레임</b>"), 0, 0)
        tg.addWidget(self.lbl_frame_no, 0, 1)
        tg.addWidget(QtWidgets.QLabel("<b>전체 경과(ms)</b>"), 0, 2)
        tg.addWidget(self.lbl_overall, 0, 3)
        tg.addWidget(QtWidgets.QLabel("<b>BAG 기록시각</b>"), 1, 0)
        tg.addWidget(self.lbl_bag_abs, 1, 1)
        tg.addWidget(QtWidgets.QLabel("경과"), 1, 2)
        tg.addWidget(self.lbl_bag_el, 1, 3)
        tg.addWidget(QtWidgets.QLabel("<b>센서 stamp</b>"), 2, 0)
        tg.addWidget(self.lbl_sensor_abs, 2, 1)
        tg.addWidget(QtWidgets.QLabel("경과"), 2, 2)
        tg.addWidget(self.lbl_sensor_el, 2, 3)
        root.addWidget(ts_box)

        # 저장 줄
        save_row = QtWidgets.QHBoxLayout()
        self.btn_save = QtWidgets.QPushButton("💾 현재 프레임 저장 (S)")
        self.btn_save.clicked.connect(self._save_current)
        self.btn_save_range = QtWidgets.QPushButton("범위 저장…")
        self.btn_save_range.clicked.connect(self._save_range)
        self.lbl_outdir = QtWidgets.QLabel(f"저장 위치: {self.output_dir}")
        self.lbl_outdir.setStyleSheet("color:#666;")
        save_row.addWidget(self.btn_save)
        save_row.addWidget(self.btn_save_range)
        save_row.addStretch(1)
        save_row.addWidget(self.lbl_outdir)
        root.addLayout(save_row)

        self.statusBar().showMessage("준비됨 — Space 재생/정지, A/D 이전·다음, S 저장")

        # 단축키
        QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self, self._toggle_play)
        QtWidgets.QShortcut(QtGui.QKeySequence("D"), self, lambda: self._step(+1))
        QtWidgets.QShortcut(QtGui.QKeySequence("Right"), self, lambda: self._step(+1))
        QtWidgets.QShortcut(QtGui.QKeySequence("A"), self, lambda: self._step(-1))
        QtWidgets.QShortcut(QtGui.QKeySequence("Left"), self, lambda: self._step(-1))
        QtWidgets.QShortcut(QtGui.QKeySequence("S"), self, self._save_current)

    # ---- 재생 로직 --------------------------------------------------------- #
    def _toggle_play(self):
        if self.playing:
            self._pause()
        else:
            self._play()

    def _play(self):
        if len(self.index) == 0:
            return
        if self.idx >= len(self.index) - 1:
            self.idx = 0
        self.playing = True
        self.btn_play.setText("⏸ 일시정지")
        self._schedule_next()

    def _pause(self):
        self.playing = False
        self.timer.stop()
        self.btn_play.setText("▶ 재생")

    def _schedule_next(self):
        if not self.playing:
            return
        delay = int(self.intervals_ms[self.idx] / self.speed)
        self.timer.start(max(1, delay))

    def _advance(self):
        if not self.playing:
            return
        if self.idx >= len(self.index) - 1:
            if self.chk_loop.isChecked():
                self.idx = 0
            else:
                self._pause()
                return
        else:
            self.idx += 1
        self._sync_widgets()
        self._update_view()
        self._schedule_next()

    def _on_speed(self, _i):
        self.speed = self.cmb_speed.currentData()

    # ---- 탐색 -------------------------------------------------------------- #
    def _step(self, delta):
        self._pause()
        self._goto(self.idx + delta)

    def _on_slider(self, val):
        if val != self.idx:
            self.idx = val
            self.spin.blockSignals(True)
            self.spin.setValue(self.idx + 1)
            self.spin.blockSignals(False)
            self._update_view()

    def _goto(self, new_idx):
        n = len(self.index)
        if n == 0:
            return
        self.idx = max(0, min(new_idx, n - 1))
        self._sync_widgets()
        self._update_view()

    def _sync_widgets(self):
        self.slider.blockSignals(True)
        self.slider.setValue(self.idx)
        self.slider.blockSignals(False)
        self.spin.blockSignals(True)
        self.spin.setValue(self.idx + 1)
        self.spin.blockSignals(False)

    # ---- 표시 -------------------------------------------------------------- #
    def _update_view(self):
        n = len(self.index)
        if n == 0:
            return
        self._show_image(self.index.get_image(self.idx))

        bag_el_str, _ = fmt_ms(self.index.bag_elapsed_ns(self.idx))
        sen_el_str, _ = fmt_ms(self.index.sensor_elapsed_ns(self.idx))
        overall_str, overall_sec = fmt_ms(self.index.bag_elapsed_ns(self.idx))

        mark = "  ✅ 저장됨" if self.idx in self.saved else ""
        self.lbl_frame_no.setText(f"{self.idx + 1} / {n}{mark}")
        self.lbl_overall.setText(f"{overall_str}  ({overall_sec:.3f} s)")
        self.lbl_bag_abs.setText(fmt_abs_ns(self.index.bag_ts[self.idx]))
        self.lbl_bag_el.setText(bag_el_str)
        self.lbl_sensor_abs.setText(fmt_abs_ns(self.index.sensor_ts[self.idx]))
        self.lbl_sensor_el.setText(sen_el_str)

        self.lbl_elapsed.setText(bag_el_str)
        self.statusBar().showMessage(
            f"frame {self.idx + 1}/{n}  ·  {overall_sec:.3f}s  ·  "
            f"{'재생중' if self.playing else '정지'}  ·  저장 {len(self.saved)}개")

    def _show_image(self, bgr):
        if bgr is None:
            self.image_label.setText("디코딩 실패")
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg).scaled(
            self.image_label.size(), QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation)
        self.image_label.setPixmap(pix)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        if len(self.index) > 0:
            self._show_image(self.index.get_image(self.idx))

    # ---- 저장 -------------------------------------------------------------- #
    def _frame_filename(self, idx):
        bag_el_str, _ = fmt_ms(self.index.bag_elapsed_ns(idx))
        safe = bag_el_str.replace(":", "-").replace(".", "_")
        bag_ts = self.index.bag_ts[idx]
        sen_ts = self.index.sensor_ts[idx]
        sen_part = f"_sensor{sen_ts}" if sen_ts is not None else ""
        return f"frame_{idx:06d}_t{safe}_bag{bag_ts}{sen_part}.{self.ext}"

    def _save_current(self):
        idx = self.idx
        fname = self._frame_filename(idx)
        cv2.imwrite(os.path.join(self.output_dir, fname), self.index.get_image(idx))
        self.saved.add(idx)
        self._update_view()
        self.statusBar().showMessage(f"저장됨: {fname}")

    def _save_range(self):
        was_playing = self.playing
        self._pause()
        n = len(self.index)
        start, ok1 = QtWidgets.QInputDialog.getInt(
            self, "범위 저장", "시작 프레임:", self.idx + 1, 1, n)
        if not ok1:
            return
        end, ok2 = QtWidgets.QInputDialog.getInt(
            self, "범위 저장", "끝 프레임:", n, start, n)
        if not ok2:
            return
        step, ok3 = QtWidgets.QInputDialog.getInt(
            self, "범위 저장", "간격 (N프레임마다):", 1, 1, n)
        if not ok3:
            return
        total = (end - start) // step + 1
        prog = QtWidgets.QProgressDialog("저장 중…", "취소", 0, total, self)
        prog.setWindowModality(QtCore.Qt.WindowModal)
        count = 0
        for i, idx in enumerate(range(start - 1, end, step)):
            if prog.wasCanceled():
                break
            cv2.imwrite(os.path.join(self.output_dir, self._frame_filename(idx)),
                        self.index.get_image(idx))
            self.saved.add(idx)
            count += 1
            prog.setValue(i + 1)
            QtWidgets.QApplication.processEvents()
        prog.close()
        self._update_view()
        QtWidgets.QMessageBox.information(
            self, "완료", f"{count}개 프레임을 저장했습니다.\n{self.output_dir}")
        if was_playing:
            self._play()

    def closeEvent(self, e):
        self._pause()
        super().closeEvent(e)


# --------------------------------------------------------------------------- #
#  진입점
# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description="ROS2 bag 이미지 플레이어 (GUI)")
    parser.add_argument("bag", nargs="?", help="rosbag2 디렉토리 경로")
    parser.add_argument("-t", "--topic", help="이미지 토픽 (미지정 시 선택창)")
    parser.add_argument("-o", "--output", default="frames", help="저장 디렉토리")
    parser.add_argument("--ext", default="png", choices=["png", "jpg"])
    parser.add_argument("--storage", choices=["sqlite3", "mcap"],
                        help="스토리지 종류 (미지정 시 자동 감지)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)

    bag_path = args.bag
    if not bag_path:
        bag_path = QtWidgets.QFileDialog.getExistingDirectory(None, "rosbag2 디렉토리 선택")
        if not bag_path:
            return

    storage_id = args.storage or detect_storage_id(bag_path)
    bag_name = os.path.basename(os.path.normpath(bag_path))

    topics = list_image_topics(bag_path, storage_id)
    if not topics:
        QtWidgets.QMessageBox.critical(None, "오류", "이미지 토픽을 찾을 수 없습니다.")
        return

    topic = args.topic
    if topic is None:
        if len(topics) == 1:
            topic = topics[0][0]
        else:
            dlg = TopicDialog(topics)
            if dlg.exec_() != QtWidgets.QDialog.Accepted or not dlg.selected:
                return
            topic = dlg.selected

    topic_type = dict(topics).get(topic)
    if topic_type is None:
        QtWidgets.QMessageBox.critical(None, "오류", f"토픽 '{topic}' 없음")
        return

    splash = QtWidgets.QProgressDialog("bag 인덱싱 중…", None, 0, 0)
    splash.setWindowTitle("로딩")
    splash.setWindowModality(QtCore.Qt.WindowModal)
    splash.show()
    QtWidgets.QApplication.processEvents()

    index = FrameIndex(bag_path, storage_id, topic, topic_type)
    splash.close()

    if len(index) == 0:
        QtWidgets.QMessageBox.critical(None, "오류", "프레임이 없습니다.")
        return

    win = PlayerWindow(index, bag_name, args.output, args.ext)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
