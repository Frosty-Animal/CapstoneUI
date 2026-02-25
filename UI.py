

"""
Scan-to-Mill UI
Presentation wireframe — dummy data, no hardware required.
Requires: PyQt6, pyvistaqt, pyvista, numpy
"""

import sys
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor

try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QGridLayout, QPushButton, QLabel, QProgressBar, QTextEdit,
        QGroupBox, QSlider, QSizePolicy, QFrame, QComboBox, QSpinBox,
        QDoubleSpinBox
    )
    from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
    from PyQt6.QtGui import QFont, QColor, QPalette, QTextCursor
except ModuleNotFoundError:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QGridLayout, QPushButton, QLabel, QProgressBar, QTextEdit,
        QGroupBox, QSlider, QSizePolicy, QFrame, QComboBox, QSpinBox,
        QDoubleSpinBox
    )
    from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
    from PyQt5.QtGui import QFont, QColor, QPalette, QTextCursor

# ── Stylesheet ────────────────────────────────────────────────────────────────
STYLE = """
QMainWindow, QWidget {
    background-color: #0d0f14;
    color: #c8cdd6;
    font-family: 'Courier New', monospace;
    font-size: 12px;
}

QGroupBox {
    border: 1px solid #2a2f3d;
    border-radius: 4px;
    margin-top: 10px;
    padding-top: 8px;
    font-size: 11px;
    font-weight: bold;
    color: #5a8fa8;
    text-transform: uppercase;
    letter-spacing: 1px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}

QPushButton {
    background-color: #1a1e28;
    border: 1px solid #2a2f3d;
    border-radius: 3px;
    padding: 7px 14px;
    color: #c8cdd6;
    font-family: 'Courier New', monospace;
    font-size: 11px;
    letter-spacing: 0.5px;
}
QPushButton:hover {
    background-color: #252a38;
    border-color: #4a8fa8;
    color: #ffffff;
}
QPushButton:pressed {
    background-color: #0d1520;
}
QPushButton:disabled {
    color: #3a3f4d;
    border-color: #1e2230;
}

QPushButton#btn_start {
    background-color: #0d2d1a;
    border-color: #1a6b3a;
    color: #2dcc70;
    font-weight: bold;
}
QPushButton#btn_start:hover {
    background-color: #133d22;
    border-color: #2dcc70;
}

QPushButton#btn_stop {
    background-color: #2d0d0d;
    border-color: #6b1a1a;
    color: #cc2d2d;
    font-weight: bold;
}
QPushButton#btn_stop:hover {
    background-color: #3d1313;
    border-color: #cc2d2d;
}

QPushButton#btn_estop {
    background-color: #3d0000;
    border: 2px solid #cc0000;
    border-radius: 4px;
    color: #ff3333;
    font-weight: bold;
    font-size: 13px;
    letter-spacing: 2px;
    padding: 10px;
}
QPushButton#btn_estop:hover {
    background-color: #5a0000;
    border-color: #ff0000;
    color: #ffffff;
}

QProgressBar {
    background-color: #111520;
    border: 1px solid #2a2f3d;
    border-radius: 2px;
    height: 16px;
    text-align: center;
    color: #c8cdd6;
    font-size: 10px;
}
QProgressBar::chunk {
    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #1a4a6b, stop:1 #2d8fa8);
    border-radius: 1px;
}

QTextEdit {
    background-color: #080a0f;
    border: 1px solid #1e2230;
    border-radius: 3px;
    color: #7a8fa0;
    font-family: 'Courier New', monospace;
    font-size: 10px;
    padding: 4px;
}

QLabel {
    color: #8a9aaa;
}
QLabel#header {
    color: #2daacc;
    font-size: 18px;
    font-weight: bold;
    letter-spacing: 3px;
}
QLabel#subheader {
    color: #4a6a7a;
    font-size: 10px;
    letter-spacing: 2px;
}
QLabel#value_display {
    background-color: #080a0f;
    border: 1px solid #1e2230;
    border-radius: 2px;
    color: #2daacc;
    font-size: 13px;
    font-weight: bold;
    padding: 4px 8px;
    min-width: 80px;
}
QLabel#status_ok {
    color: #2dcc70;
    font-weight: bold;
}
QLabel#status_off {
    color: #cc4d2d;
    font-weight: bold;
}

QSlider::groove:horizontal {
    background: #1a1e28;
    height: 4px;
    border-radius: 2px;
}
QSlider::handle:horizontal {
    background: #2d8fa8;
    width: 12px;
    height: 12px;
    margin: -4px 0;
    border-radius: 6px;
}
QSlider::sub-page:horizontal {
    background: #2d8fa8;
    border-radius: 2px;
}

QComboBox {
    background-color: #1a1e28;
    border: 1px solid #2a2f3d;
    border-radius: 3px;
    padding: 4px 8px;
    color: #c8cdd6;
}
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView {
    background-color: #1a1e28;
    border: 1px solid #2a2f3d;
    selection-background-color: #252a38;
}

QSpinBox, QDoubleSpinBox {
    background-color: #1a1e28;
    border: 1px solid #2a2f3d;
    border-radius: 3px;
    padding: 4px 6px;
    color: #c8cdd6;
}

QFrame#divider {
    color: #2a2f3d;
}
"""


# ── Dummy point cloud generator ───────────────────────────────────────────────
def generate_dummy_pointcloud(n_points: int = 4000, seed: int = 0) -> pv.PolyData:
    rng = np.random.default_rng(seed)
    # Simulate scanning a roughly box-shaped object
    u = rng.uniform(0, 2 * np.pi, n_points)
    v = rng.uniform(0, np.pi, n_points)
    r = rng.uniform(0.8, 1.0, n_points)
    x = r * np.sin(v) * np.cos(u) * 50 + rng.normal(0, 1.5, n_points)
    y = r * np.sin(v) * np.sin(u) * 40 + rng.normal(0, 1.5, n_points)
    z = r * np.cos(v) * 30 + rng.normal(0, 1.5, n_points)
    pts = np.column_stack([x, y, z])
    cloud = pv.PolyData(pts)
    cloud["depth"] = z
    return cloud


# ── Worker thread for fake scan progress ─────────────────────────────────────
class ScanWorker(QThread):
    progress = pyqtSignal(int)
    point_count = pyqtSignal(int)
    log_message = pyqtSignal(str)
    finished = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._paused = False
        self._stopped = False

    def pause(self): self._paused = True
    def resume(self): self._paused = False
    def stop(self): self._stopped = True; self._paused = False

    def run(self):
        steps = 100
        self.log_message.emit("[SCAN] Initializing depth sensor...")
        self.msleep(400)
        self.log_message.emit("[SCAN] Stream started at 640x480 @ 30fps")
        for i in range(1, steps + 1):
            while self._paused:
                self.msleep(100)
            if self._stopped:
                self.log_message.emit("[SCAN] Scan aborted by user.")
                return
            self.msleep(80)
            self.progress.emit(i)
            self.point_count.emit(i * 47)
            if i == 25:
                self.log_message.emit("[SCAN] 25% — front face captured")
            elif i == 50:
                self.log_message.emit("[SCAN] 50% — rotating to side profile")
            elif i == 75:
                self.log_message.emit("[SCAN] 75% — top surface acquired")
            elif i == 100:
                self.log_message.emit("[SCAN] Complete — 4700 points captured")
        self.finished.emit()


# ── Main Window ───────────────────────────────────────────────────────────────
class ScanToMillUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SCAN-TO-MILL  //  Control Interface")
        self.setMinimumSize(1280, 820)
        self.showMaximized()
        self._scan_worker = None
        self._scan_running = False
        self._scan_paused = False
        self._point_count = 0
        self._build_ui()
        self._init_viewport()
        self._start_clock()

    # ── Layout ────────────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        root.addWidget(self._make_header())

        body = QHBoxLayout()
        body.setSpacing(10)
        body.addWidget(self._make_left_panel(), stretch=0)
        body.addWidget(self._make_viewport_panel(), stretch=1)
        body.addWidget(self._make_right_panel(), stretch=0)
        root.addLayout(body, stretch=1)

        root.addWidget(self._make_bottom_panel())

    def _make_header(self):
        w = QWidget()
        w.setFixedHeight(52)
        w.setStyleSheet("background:#080a0f; border-bottom:1px solid #1e2230;")
        lay = QHBoxLayout(w)
        lay.setContentsMargins(12, 4, 12, 4)

        title = QLabel("SCAN-TO-MILL")
        title.setObjectName("header")
        sub = QLabel("3D CAPTURE → CNC MACHINING SYSTEM  //  OSU CAPSTONE")
        sub.setObjectName("subheader")

        lay.addWidget(title)
        lay.addWidget(sub)
        lay.addStretch()

        self.lbl_clock = QLabel("--:--:--")
        self.lbl_clock.setObjectName("value_display")
        self.lbl_camera_status = QLabel("● CAMERA OFFLINE")
        self.lbl_camera_status.setObjectName("status_off")
        self.lbl_cnc_status = QLabel("● CNC OFFLINE")
        self.lbl_cnc_status.setObjectName("status_off")

        for w2 in [self.lbl_camera_status, self.lbl_cnc_status, self.lbl_clock]:
            lay.addWidget(w2)
            lay.addSpacing(14)
        return w

    def _make_left_panel(self):
        w = QWidget()
        w.setFixedWidth(210)
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        # ── Scan Controls ──
        grp = QGroupBox("Scan Controls")
        g = QVBoxLayout(grp)

        self.btn_start = QPushButton("▶  START SCAN")
        self.btn_start.setObjectName("btn_start")
        self.btn_pause = QPushButton("⏸  PAUSE")
        self.btn_pause.setEnabled(False)
        self.btn_stop = QPushButton("■  STOP")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.setEnabled(False)

        self.btn_start.clicked.connect(self._on_start)
        self.btn_pause.clicked.connect(self._on_pause)
        self.btn_stop.clicked.connect(self._on_stop)

        for b in [self.btn_start, self.btn_pause, self.btn_stop]:
            g.addWidget(b)

        g.addSpacing(6)
        g.addWidget(self._labeled_combo("Resolution", ["640×480", "1280×720", "848×480"]))
        g.addWidget(self._labeled_combo("Scan Mode", ["Depth Only", "Depth + RGB", "IR"]))

        lbl_pts = QLabel("Points Captured")
        self.lbl_point_count = QLabel("0")
        self.lbl_point_count.setObjectName("value_display")
        g.addWidget(lbl_pts)
        g.addWidget(self.lbl_point_count)

        lay.addWidget(grp)

        # ── Post-Processing ──
        grp2 = QGroupBox("Post-Process")
        g2 = QVBoxLayout(grp2)
        self.btn_filter = QPushButton("Remove Outliers")
        self.btn_mesh = QPushButton("Generate Mesh")
        self.btn_export = QPushButton("Export STL")
        for b in [self.btn_filter, self.btn_mesh, self.btn_export]:
            b.setEnabled(False)
            g2.addWidget(b)
        lay.addWidget(grp2)

        lay.addStretch()
        return w

    def _make_viewport_panel(self):
        grp = QGroupBox("3D Viewport")
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(4, 12, 4, 4)

        toolbar = QHBoxLayout()
        self.btn_view_cloud = QPushButton("Point Cloud")
        self.btn_view_mesh = QPushButton("Mesh")
        self.btn_view_reset = QPushButton("Reset Camera")
        self.lbl_render_mode = QLabel("MODE: POINT CLOUD")
        self.lbl_render_mode.setObjectName("value_display")

        self.btn_view_cloud.clicked.connect(lambda: self._set_view_mode("cloud"))
        self.btn_view_mesh.clicked.connect(lambda: self._set_view_mode("mesh"))
        self.btn_view_reset.clicked.connect(self._reset_camera)

        for w2 in [self.btn_view_cloud, self.btn_view_mesh, self.btn_view_reset, self.lbl_render_mode]:
            toolbar.addWidget(w2)
        toolbar.addStretch()
        lay.addLayout(toolbar)

        # PyVista interactor placeholder (populated in _init_viewport)
        self.vtk_frame = QFrame()
        self.vtk_frame.setStyleSheet("background:#050709; border:1px solid #1e2230;")
        self.vtk_frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        lay.addWidget(self.vtk_frame, stretch=1)

        return grp

    def _make_right_panel(self):
        w = QWidget()
        w.setFixedWidth(210)
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        # ── CNC Controls ──
        grp = QGroupBox("CNC Controls")
        g = QVBoxLayout(grp)

        self.btn_home = QPushButton("⌂  Home All Axes")
        self.btn_send_gcode = QPushButton("▶  Run G-Code")
        self.btn_send_gcode.setEnabled(False)
        self.btn_pause_cnc = QPushButton("⏸  Pause Job")
        self.btn_pause_cnc.setEnabled(False)

        for b in [self.btn_home, self.btn_send_gcode, self.btn_pause_cnc]:
            g.addWidget(b)

        g.addSpacing(6)

        # DRO
        for axis, val in [("X", "0.000"), ("Y", "0.000"), ("Z", "0.000")]:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"  {axis}:"))
            lbl = QLabel(val + " mm")
            lbl.setObjectName("value_display")
            setattr(self, f"dro_{axis}", lbl)
            row.addWidget(lbl)
            g.addLayout(row)

        g.addSpacing(6)
        g.addWidget(QLabel("Feed Rate Override"))
        self.slider_feed = QSlider(Qt.Horizontal)
        self.slider_feed.setRange(0, 200)
        self.slider_feed.setValue(100)
        self.lbl_feed_val = QLabel("100 %")
        self.lbl_feed_val.setObjectName("value_display")
        self.slider_feed.valueChanged.connect(lambda v: self.lbl_feed_val.setText(f"{v} %"))
        g.addWidget(self.slider_feed)
        g.addWidget(self.lbl_feed_val)

        g.addSpacing(6)
        g.addWidget(QLabel("Spindle RPM"))
        self.slider_rpm = QSlider(Qt.Horizontal)
        self.slider_rpm.setRange(0, 24000)
        self.slider_rpm.setValue(12000)
        self.lbl_rpm_val = QLabel("12000 RPM")
        self.lbl_rpm_val.setObjectName("value_display")
        self.slider_rpm.valueChanged.connect(lambda v: self.lbl_rpm_val.setText(f"{v} RPM"))
        g.addWidget(self.slider_rpm)
        g.addWidget(self.lbl_rpm_val)

        lay.addWidget(grp)

        # ── E-Stop ──
        self.btn_estop = QPushButton("⚠  EMERGENCY STOP")
        self.btn_estop.setObjectName("btn_estop")
        self.btn_estop.setFixedHeight(54)
        self.btn_estop.clicked.connect(self._on_estop)
        lay.addWidget(self.btn_estop)

        lay.addStretch()
        return w

    def _make_bottom_panel(self):
        w = QWidget()
        w.setFixedHeight(160)
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        # Progress section
        prog_grp = QGroupBox("Job Progress")
        prog_lay = QVBoxLayout(prog_grp)

        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("%p%  —  IDLE")
        prog_lay.addWidget(self.progress_bar)

        stats_row = QHBoxLayout()
        for label, attr in [("Elapsed", "lbl_elapsed"), ("Est. Remaining", "lbl_remaining"),
                             ("Points", "lbl_pts_stat"), ("Stage", "lbl_stage")]:
            col = QVBoxLayout()
            col.addWidget(QLabel(label))
            lbl = QLabel("—")
            lbl.setObjectName("value_display")
            setattr(self, attr, lbl)
            col.addWidget(lbl)
            stats_row.addLayout(col)
        prog_lay.addLayout(stats_row)
        lay.addWidget(prog_grp, stretch=1)

        # Log section
        log_grp = QGroupBox("System Log")
        log_lay = QVBoxLayout(log_grp)
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        log_lay.addWidget(self.log_output)
        lay.addWidget(log_grp, stretch=1)

        self._log("[SYS] Scan-to-Mill UI initialized.")
        self._log("[SYS] Waiting for hardware connection...")
        return w

    # ── Viewport ──────────────────────────────────────────────────────────────
    def _init_viewport(self):
        vtk_lay = QVBoxLayout(self.vtk_frame)
        vtk_lay.setContentsMargins(0, 0, 0, 0)

        self.plotter = QtInteractor(self.vtk_frame)
        self.plotter.set_background("#050709")
        vtk_lay.addWidget(self.plotter.interactor)

        # Load dummy point cloud
        self._cloud = generate_dummy_pointcloud(2000)
        self.plotter.add_mesh(
            self._cloud,
            scalars="depth",
            cmap="cool",
            point_size=3,
            render_points_as_spheres=True,
            name="pointcloud",
            show_scalar_bar=False,
        )
        self.plotter.add_axes(color="#4a6a7a")
        self.plotter.camera_position = "iso"
        self._view_mode = "cloud"
        self._log("[VIZ] Dummy point cloud loaded (2000 pts)")

    def _set_view_mode(self, mode):
        self._view_mode = mode
        if mode == "cloud":
            self.lbl_render_mode.setText("MODE: POINT CLOUD")
            self.plotter.remove_actor("mesh_actor")
            self.plotter.add_mesh(
                self._cloud, scalars="depth", cmap="cool",
                point_size=3, render_points_as_spheres=True,
                name="pointcloud", show_scalar_bar=False
            )
        else:
            self.lbl_render_mode.setText("MODE: MESH")
            self.plotter.remove_actor("pointcloud")
            surf = self._cloud.reconstruct_surface(nbr_sz=10)
            self.plotter.add_mesh(
                surf, color="#2a5a7a", show_edges=False,
                opacity=1.0, name="mesh_actor",
                pbr=False, interpolate_before_map=False
)
        self.plotter.render()

    def _reset_camera(self):
        self.plotter.camera_position = "iso"
        self.plotter.render()

    def _update_pointcloud_live(self, n_points: int):
        new_cloud = generate_dummy_pointcloud(n_points, seed=n_points)
        self._cloud = new_cloud
        self.plotter.remove_actor("pointcloud")
        self.plotter.add_mesh(
            self._cloud, scalars="depth", cmap="cool",
            point_size=3, render_points_as_spheres=True,
            name="pointcloud", show_scalar_bar=False
        )
        self.plotter.render()

    # ── Scan Actions ──────────────────────────────────────────────────────────
    def _on_start(self):
        self._scan_running = True
        self._scan_paused = False
        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_stop.setEnabled(True)
        self.btn_filter.setEnabled(False)
        self.btn_mesh.setEnabled(False)
        self.btn_export.setEnabled(False)
        self.progress_bar.setFormat("%p%  —  SCANNING")
        self.lbl_stage.setText("SCANNING")
        self._scan_elapsed = 0
        self._scan_timer = QTimer()
        self._scan_timer.timeout.connect(self._tick_elapsed)
        self._scan_timer.start(1000)

        self._scan_worker = ScanWorker()
        self._scan_worker.progress.connect(self._on_progress)
        self._scan_worker.point_count.connect(self._on_points)
        self._scan_worker.log_message.connect(self._log)
        self._scan_worker.finished.connect(self._on_scan_done)
        self._scan_worker.start()

    def _on_pause(self):
        if not self._scan_paused:
            self._scan_paused = True
            self._scan_worker.pause()
            self.btn_pause.setText("▶  RESUME")
            self.progress_bar.setFormat("%p%  —  PAUSED")
            self.lbl_stage.setText("PAUSED")
            self._log("[SCAN] Paused.")
        else:
            self._scan_paused = False
            self._scan_worker.resume()
            self.btn_pause.setText("⏸  PAUSE")
            self.progress_bar.setFormat("%p%  —  SCANNING")
            self.lbl_stage.setText("SCANNING")
            self._log("[SCAN] Resumed.")

    def _on_stop(self):
        if self._scan_worker:
            self._scan_worker.stop()
        self._reset_scan_ui()
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("%p%  —  IDLE")
        self.lbl_stage.setText("IDLE")

    def _on_scan_done(self):
        self._reset_scan_ui()
        self.progress_bar.setFormat("100%  —  COMPLETE")
        self.lbl_stage.setText("COMPLETE")
        self.btn_filter.setEnabled(True)
        self.btn_mesh.setEnabled(True)
        self.btn_export.setEnabled(True)
        self._log("[SYS] Post-processing options unlocked.")

    def _reset_scan_ui(self):
        self._scan_running = False
        self._scan_paused = False
        if hasattr(self, "_scan_timer"):
            self._scan_timer.stop()
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_pause.setText("⏸  PAUSE")
        self.btn_stop.setEnabled(False)

    def _on_progress(self, val: int):
        self.progress_bar.setValue(val)
        remaining = int((100 - val) * 0.08)
        self.lbl_remaining.setText(f"{remaining}s")

    def _on_points(self, n: int):
        self._point_count = n
        self.lbl_point_count.setText(str(n))
        self.lbl_pts_stat.setText(str(n))
        if n % 500 < 50:
            self._update_pointcloud_live(min(n, 4700))

    def _tick_elapsed(self):
        self._scan_elapsed += 1
        m, s = divmod(self._scan_elapsed, 60)
        self.lbl_elapsed.setText(f"{m:02d}:{s:02d}")

    # ── E-Stop ────────────────────────────────────────────────────────────────
    def _on_estop(self):
        self._on_stop()
        self._log("[!!!] EMERGENCY STOP TRIGGERED — all motion halted.")
        self.lbl_stage.setText("E-STOP")
        self.progress_bar.setFormat("E-STOP")
        self.progress_bar.setStyleSheet(
            "QProgressBar::chunk { background-color: #8b0000; }"
        )

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}]  {msg}"
        self.log_output.append(line)
        self.log_output.moveCursor(QTextCursor.MoveOperation.End)

    def _labeled_combo(self, label: str, items: list) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 2, 0, 2)
        lay.setSpacing(2)
        lay.addWidget(QLabel(label))
        cb = QComboBox()
        cb.addItems(items)
        lay.addWidget(cb)
        return w

    def _start_clock(self):
        timer = QTimer(self)
        timer.timeout.connect(self._update_clock)
        timer.start(1000)
        self._update_clock()

    def _update_clock(self):
        from datetime import datetime
        self.lbl_clock.setText(datetime.now().strftime("%H:%M:%S"))

    def closeEvent(self, event):
        if self._scan_worker and self._scan_worker.isRunning():
            self._scan_worker.stop()
            self._scan_worker.wait()
        self.plotter.close()
        event.accept()


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    pv.set_plot_theme("dark")
    pv.global_theme.multi_samples = 1        # <-- add this
    pv.global_theme.smooth_shading = False
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    window = ScanToMillUI()
    window.show()
    sys.exit(app.exec())
