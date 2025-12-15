"""Pinocchio GUI Wrapper (PyQt6 + meshcat)."""

import logging
import sys
import os


import meshcat.geometry as g
import meshcat.visualizer as viz
import numpy as np
import pinocchio as pin
from PyQt6 import QtCore, QtWidgets, QtGui

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class LogPanel(QtWidgets.QTextEdit):
    """Log panel widget for displaying messages."""

    def __init__(self) -> None:
        """Initialize the log panel."""
        super().__init__()
        self.setReadOnly(True)
        self.setStyleSheet(
            "background:#111; color:#0F0; font-family:Consolas; font-size:12px;"
        )


class PinocchioGUI(QtWidgets.QMainWindow):
    """Main GUI widget for Pinocchio robot visualization and computation."""

    def __init__(self) -> None:
        """Initialize the Pinocchio GUI."""
        super().__init__()
        self.setWindowTitle("Pinocchio Golf Model (Dynamics & Kinematics)")
        self.resize(600, 800)

        # Internal state
        self.model: Optional[pin.Model] = None
        self.data: Optional[pin.Data] = None
        self.q: Optional[np.ndarray] = None
        self.v: Optional[np.ndarray] = None

        self.joint_sliders: List[QtWidgets.QSlider] = []
        self.joint_spinboxes: List[QtWidgets.QDoubleSpinBox] = []
        self.joint_names: List[str] = []

        self.operating_mode = "dynamic"  # "dynamic", "kinematic"
        self.is_running = False
        self.dt = 0.01

        # Meshcat viewer
        # Using open_window=True trying to open default browser
        self.viewer = viz.Visualizer()  # Let it find port
        # self.viewer.open() # Opens browser

        logger.info(f"Meshcat URL: {self.viewer.url}")

        # Setup UI
        self._setup_ui()

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._game_loop)
        self.timer.start(int(self.dt * 1000))

        # Try load default model
        default_urdf = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "../../models/generated/golfer.urdf"
            )
        )
        if os.path.exists(default_urdf):
            self.load_urdf(default_urdf)

    def _setup_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # 1. Top Bar: Load & Mode
        top_layout = QtWidgets.QHBoxLayout()

        self.load_btn = QtWidgets.QPushButton("Load URDF")
        self.load_btn.clicked.connect(lambda: self.load_urdf())
        top_layout.addWidget(self.load_btn)

        top_layout.addStretch()

        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["Dynamic (Physics)", "Kinematic (Pose)"])
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        top_layout.addWidget(QtWidgets.QLabel("Mode:"))
        top_layout.addWidget(self.mode_combo)

        layout.addLayout(top_layout)

        # 2. Controls Stack
        self.controls_stack = QtWidgets.QStackedWidget()
        layout.addWidget(self.controls_stack)

        # -- Page 1: Dynamic
        dyn_page = QtWidgets.QWidget()
        dyn_layout = QtWidgets.QVBoxLayout(dyn_page)
        self.btn_run = QtWidgets.QPushButton("Run Simulation")
        self.btn_run.setCheckable(True)
        self.btn_run.clicked.connect(self._toggle_run)
        dyn_layout.addWidget(self.btn_run)

        self.btn_reset = QtWidgets.QPushButton("Reset")
        self.btn_reset.clicked.connect(self._reset_simulation)
        dyn_layout.addWidget(self.btn_reset)

        dyn_layout.addStretch()
        self.controls_stack.addWidget(dyn_page)

        # -- Page 2: Kinematic
        kin_page = QtWidgets.QWidget()
        kin_layout = QtWidgets.QVBoxLayout(kin_page)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        self.slider_container = QtWidgets.QWidget()
        self.slider_layout = QtWidgets.QVBoxLayout(self.slider_container)
        scroll.setWidget(self.slider_container)

        kin_layout.addWidget(scroll)
        self.controls_stack.addWidget(kin_page)

        # 3. Visuals & Logs
        vis_group = QtWidgets.QGroupBox("Visualization")
        vis_layout = QtWidgets.QHBoxLayout()

        self.chk_frames = QtWidgets.QCheckBox("Show Frames")
        self.chk_frames.toggled.connect(self._toggle_frames)
        vis_layout.addWidget(self.chk_frames)

        self.chk_coms = QtWidgets.QCheckBox("Show COMs")
        self.chk_coms.toggled.connect(self._toggle_coms)
        vis_layout.addWidget(self.chk_coms)

        vis_group.setLayout(vis_layout)
        layout.addWidget(vis_group)

        self.log = LogPanel()
        layout.addWidget(self.log)

    def log_write(self, text: str) -> None:
        self.log.append(text)
        logger.info(text)

    def load_urdf(self, fname: Optional[str] = None) -> None:
        if not fname:
            fname, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Select URDF File", "", "URDF Files (*.urdf *.xml)"
            )

        if not fname:
            return

        try:
            self.model = pin.buildModelFromUrdf(fname)
            if self.model is None:
                raise RuntimeError("Failed to build model from URDF")

            self.data = self.model.createData()
            self.q = pin.neutral(self.model)
            self.v = np.zeros(self.model.nv)

            # Load visual into Meshcat
            # Careful: meshcat URDF loader is separate from Pinocchio
            self.viewer["robot"].delete()
            self.viewer["robot"].set_object(g.URDFLoader().load(fname))

            self.log_write(f"Loaded URDF: {fname}")
            self.log_write(f"NQ: {self.model.nq}, NV: {self.model.nv}")

            # Rebuild Kinematic Controls
            self._build_kinematic_controls()

            # Init state
            self._update_viewer()

        except Exception as e:
            self.log_write(f"Error loading URDF: {e}")

    def _build_kinematic_controls(self) -> None:
        if self.model is None:
            return

        # Clear layout
        while self.slider_layout.count():
            item = self.slider_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        self.joint_sliders = []
        self.joint_spinboxes = []
        self.joint_names = []

        # Iterate joints (skip universe)
        for i in range(1, self.model.njoints):
            joint_name = self.model.names[i]
            # Simple assumption: 1 DOF per joint for sliders.
            # If multi-dof (spherical), sliders might be tricky without mapping.
            # We assume standard revolute/prismatic for UI.
            idx_q = self.model.joints[i].idx_q
            nq_joint = self.model.joints[i].nq

            if nq_joint != 1:
                # Skip multi-dof joints for simple slider UI for now
                continue

            self.joint_names.append(joint_name)

            row = QtWidgets.QWidget()
            r_layout = QtWidgets.QHBoxLayout(row)
            r_layout.setContentsMargins(0, 0, 0, 0)

            r_layout.addWidget(QtWidgets.QLabel(f"{joint_name}:"))

            slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            slider.setRange(-314, 314)
            slider.setValue(0)

            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-10.0, 10.0)
            spin.setSingleStep(0.1)

            # Connect
            idx = int(idx_q)  # Capture index into q vector

            slider.valueChanged.connect(
                lambda val, s=spin, k=idx: self._on_slider(val, s, k)
            )
            spin.valueChanged.connect(
                lambda val, s=slider, k=idx: self._on_spin(val, s, k)
            )

            r_layout.addWidget(slider)
            r_layout.addWidget(spin)
            self.slider_layout.addWidget(row)

            self.joint_sliders.append(slider)
            self.joint_spinboxes.append(spin)

    def _on_slider(self, val: int, spin: QtWidgets.QDoubleSpinBox, idx: int) -> None:
        angle = val / 100.0
        spin.blockSignals(True)
        spin.setValue(angle)
        spin.blockSignals(False)
        self._update_q(idx, angle)

    def _on_spin(self, val: float, slider: QtWidgets.QSlider, idx: int) -> None:
        slider.blockSignals(True)
        slider.setValue(int(val * 100))
        slider.blockSignals(False)
        self._update_q(idx, val)

    def _update_q(self, idx: int, val: float) -> None:
        if self.operating_mode != "kinematic":
            return
        if self.q is not None:
            self.q[idx] = val
            self._update_viewer()

    def _update_viewer(self) -> None:
        if self.model is None or self.data is None or self.q is None:
            return

        # Update Visuals
        q_dict = {}
        for i in range(1, self.model.njoints):
            j = self.model.joints[i]
            if j.nq == 1:
                # Map to joint name, not q index
                q_dict[self.model.names[i]] = self.q[j.idx_q]

        self.viewer["robot"].set_joint_positions(q_dict)

        # Kinematics Logic for frames
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        if self.chk_coms.isChecked():
            pin.centerOfMass(self.model, self.data, self.q)

        # Overlays
        if self.chk_frames.isChecked():
            self._draw_frames()
        if self.chk_coms.isChecked():
            self._draw_coms()

    def _draw_frames(self) -> None:
        if self.model is None or self.data is None:
            return

        # Visualize only Body frames (typically associated with joints)
        # To avoid clutter, maybe just frames with mass?
        # For now, visualizing operational frames (oMf)
        # Creating objects every frame is slow in Meshcat over ZMQ.
        # Better: create once, SetTransform.
        # For simplicity in this script:
        for i, frame in enumerate(self.model.frames):
            if frame.name == "universe":
                continue
            # Setup triad if not exists (check by checking if we made it before?)
            # Just setting object with transform is what meshcat expects.
            # Use SetTransform if object exists.
            # Python meshcat client handles caching if same object.

            T = self.data.oMf[i]
            # Convert T (pin.SE3) to replacement
            # T.translation, T.rotation

            # Meshcat uses 4x4 flattened matrix in column major?
            # Or direct translation/rotation.
            # set_transform(matrix)
            M = T.homogeneous
            self.viewer[f"overlays/frames/{frame.name}"].set_transform(M)

    def _draw_coms(self) -> None:
        if self.model is None or self.data is None:
            return

        # Center of Mass of the subtree or individual bodies?
        # data.com is vector of COMs... wait, pin.centerOfMass computes global COM (hg) or per-link?
        # pin.centerOfMass(model, data, q) computes data.com[0] (Total COM).
        # To get individual body COMs, we use data.com[i] which are subtree COMs? No.
        # data.com[i] is COM of subtree starting at joint i?
        # Actually `data.oMi` is placement of joint i.
        # Body COM is relative to joint.
        # Let's use `data.com` properly. `centerOfMass(model,data,q)` computes `data.com[0]`.
        # `jacobianCenterOfMass`...
        # Function `forwardKinematics` computes `data.oMi`.
        # Body COM in world: oMi * local_com.

        for i in range(1, self.model.njoints):
            # Joint i. Associated body has inertia.
            inertia = self.model.inertias[i]
            oMi = self.data.oMi[i]
            com_world = oMi * inertia.lever

            # Draw Sphere
            self.viewer[f"overlays/coms/{self.model.names[i]}"].set_transform(
                pin.SE3(np.eye(3), com_world).homogeneous
            )

    # --- Vis Helpers ---
    def _toggle_frames(self, checked: bool) -> None:
        if not checked:
            self.viewer["overlays/frames"].delete()
        else:
            # Create objects once
            if self.model:
                for frame in self.model.frames:
                    if frame.name == "universe":
                        continue
                    # Triad
                    self.viewer[f"overlays/frames/{frame.name}"].set_object(
                        g.Triad(scale=0.1)
                    )
            self._update_viewer()

    def _toggle_coms(self, checked: bool) -> None:
        if not checked:
            self.viewer["overlays/coms"].delete()
        else:
            if self.model:
                for i in range(1, self.model.njoints):
                    self.viewer[f"overlays/coms/{self.model.names[i]}"].set_object(
                        g.Sphere(0.02), g.MeshLambertMaterial(color=0xFFFF00)
                    )
            self._update_viewer()


def main() -> None:
    """Main entry point for the GUI application."""
    app = QtWidgets.QApplication(sys.argv)
    window = PinocchioGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
