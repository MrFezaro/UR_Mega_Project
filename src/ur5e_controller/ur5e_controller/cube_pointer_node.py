import json
import threading
import time
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── Set True to print every raw detection frame to the console ────────────────
DEBUG_DETECTIONS = False

COLOR_ORDER = ['Red', 'Green', 'Blue']


def _rotvec_to_matrix(rotvec):
    rv = np.asarray(rotvec, dtype=float)
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.eye(3)
    k = rv / angle
    c, s = np.cos(angle), np.sin(angle)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return c * np.eye(3) + (1 - c) * np.outer(k, k) + s * K


class State(Enum):
    HOME     = auto()   # movej back to overview pose
    DETECT   = auto()   # look for current color from overview
    SEARCH   = auto()   # rotate ±45° and look again
    APPROACH = auto()   # movej to 10 cm above the cube
    RETREAT  = auto()   # advance to next color
    DONE     = auto()   # all three found/skipped — restart


class CubePointerNode(Node):

    def __init__(self):
        super().__init__('cube_pointer')

        # Full TCP pose at home/overview position [x, y, z, rx, ry, rz]
        self.declare_parameter('home_pose',
            [-0.28297, -0.47154, 0.64359, -1.207, -2.896, -0.010])
        self.declare_parameter('motion_duration',   6.0)
        self.declare_parameter('approach_height',   0.10)
        self.declare_parameter('table_z',          -0.016)
        self.declare_parameter('cube_height',       0.05)
        self.declare_parameter('camera_yaw_deg',    180.0)
        # Degrees to rotate the camera position around the robot base Z axis for search
        self.declare_parameter('search_angles_deg', [45.0, -45.0])
        # Camera intrinsics — no calibration file, so set these manually.
        # For 640x480 with ~70° FOV: fx=fy≈500. Tune if XY position is off.
        self.declare_parameter('camera_fx', 500.0)
        self.declare_parameter('camera_fy', 500.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)

        self._urscript_pub = self.create_publisher(
            String, '/urscript_interface/script_command', 1)

        self._det_lock = threading.Lock()
        self._latest_det = None
        self.create_subscription(String, '/cube_detections', self._on_detection, 10)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _on_detection(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if DEBUG_DETECTIONS:
            self.get_logger().info(f'[det] {data}')
        with self._det_lock:
            self._latest_det = data

    # ── helpers ────────────────────────────────────────────────────────────────

    def _flush(self):
        with self._det_lock:
            self._latest_det = None

    def _wait_detection(self, timeout=6.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            with self._det_lock:
                if self._latest_det is not None:
                    det = self._latest_det
                    self._latest_det = None
                    return det
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        return None

    def _pixel_to_robot(self, u: float, v: float, tcp_xyz):
        """Project pixel (u,v) onto the table plane using the given TCP position."""
        fx = float(self.get_parameter('camera_fx').value)
        fy = float(self.get_parameter('camera_fy').value)
        cx = float(self.get_parameter('camera_cx').value)
        cy = float(self.get_parameter('camera_cy').value)

        tcp_xyz = np.asarray(tcp_xyz, dtype=float)
        home_pose = list(self.get_parameter('home_pose').value)
        tcp_rv  = np.array(home_pose[3:], dtype=float)
        table_z = float(self.get_parameter('table_z').value)
        yaw     = np.radians(float(self.get_parameter('camera_yaw_deg').value))

        R_tool = _rotvec_to_matrix(tcp_rv)
        cy_a, sy_a = np.cos(yaw), np.sin(yaw)
        R_cam = np.array([[cy_a, -sy_a, 0.],
                          [sy_a,  cy_a, 0.],
                          [0.,    0.,   1.]])

        ray_cam  = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        ray_base = R_tool @ R_cam @ ray_cam

        if abs(ray_base[2]) < 1e-6 or (table_z - tcp_xyz[2]) / ray_base[2] < 0:
            return None

        t  = (table_z - tcp_xyz[2]) / ray_base[2]
        pt = tcp_xyz + t * ray_base
        return float(pt[0]), float(pt[1]), table_z

    def _find_color(self, det: dict, color: str, tcp_xyz):
        px = det.get(color)
        if px is None:
            return None
        return self._pixel_to_robot(px[0], px[1], tcp_xyz)

    # ── URScript ───────────────────────────────────────────────────────────────

    def _send(self, script: str):
        msg = String()
        msg.data = script
        self._urscript_pub.publish(msg)

    def _movej_pose(self, pose: list, duration: float):
        """Joint-space move to a Cartesian TCP pose — robot solves IK."""
        p = ', '.join(f'{x:.6f}' for x in pose)
        script = f'movej(p[{p}], a=1.2, v=1.05, t={duration})\n'
        self.get_logger().debug(f'>> {script.strip()}')
        self._send(script)
        time.sleep(duration + 1.0)

    # ── state machine ──────────────────────────────────────────────────────────

    def _run(self):
        time.sleep(1.0)

        home_pose    = list(self.get_parameter('home_pose').value)
        duration     = float(self.get_parameter('motion_duration').value)
        height       = float(self.get_parameter('approach_height').value)
        table_z      = float(self.get_parameter('table_z').value)
        cube_h       = float(self.get_parameter('cube_height').value)
        s_angles_deg = list(self.get_parameter('search_angles_deg').value)

        home_xyz  = home_pose[:3]               # [x, y, z]
        orient    = home_pose[3:]               # [rx, ry, rz] — reuse for all moves
        approach_z = table_z + cube_h + height  # absolute Z to hover above a cube

        # Search poses: rotate home XY ±45° around base Z axis, keep same height
        home_r   = float(np.hypot(home_xyz[0], home_xyz[1]))
        home_phi = float(np.arctan2(home_xyz[1], home_xyz[0]))
        search_poses = []
        for deg in s_angles_deg:
            phi = home_phi + np.radians(deg)
            sx, sy = home_r * np.cos(phi), home_r * np.sin(phi)
            search_poses.append([sx, sy, home_xyz[2]] + orient)

        self.get_logger().info('Starting.')

        color_idx  = 0
        search_idx = 0
        target_pos = None
        state      = State.HOME

        while rclpy.ok():
            color = COLOR_ORDER[color_idx]

            # ── HOME ──────────────────────────────────────────────────────────
            if state is State.HOME:
                self.get_logger().info(f'--- {color} ---')
                self._movej_pose(home_pose, duration)
                self._flush()
                state = State.DETECT

            # ── DETECT ────────────────────────────────────────────────────────
            elif state is State.DETECT:
                det = self._wait_detection(timeout=6.0)
                target_pos = self._find_color(det, color, home_xyz) if det else None
                if target_pos is not None:
                    state = State.APPROACH
                else:
                    search_idx = 0
                    state = State.SEARCH

            # ── SEARCH ────────────────────────────────────────────────────────
            elif state is State.SEARCH:
                if search_idx >= len(search_poses):
                    self.get_logger().warn(f'{color}: not found — skipping.')
                    state = State.RETREAT
                else:
                    deg = s_angles_deg[search_idx]
                    sp  = search_poses[search_idx]
                    self.get_logger().info(f'{color}: searching at {deg:+.0f}°')
                    self._movej_pose(sp, duration)
                    self._flush()
                    det = self._wait_detection(timeout=5.0)
                    search_idx += 1
                    target_pos = self._find_color(det, color, sp[:3]) if det else None
                    if target_pos is not None:
                        state = State.APPROACH

            # ── APPROACH ──────────────────────────────────────────────────────
            elif state is State.APPROACH:
                rx, ry, _ = target_pos
                self.get_logger().info(
                    f'{color}: pointing at ({rx:.3f}, {ry:.3f}, {approach_z:.3f})')
                self._movej_pose([rx, ry, approach_z] + orient, duration)
                self.get_logger().info(f'{color}: done.')
                state = State.RETREAT

            # ── RETREAT ───────────────────────────────────────────────────────
            elif state is State.RETREAT:
                color_idx += 1
                if color_idx >= len(COLOR_ORDER):
                    state = State.DONE
                else:
                    state = State.HOME

            # ── DONE ──────────────────────────────────────────────────────────
            elif state is State.DONE:
                self.get_logger().info('=== All done. Restarting. ===')
                color_idx = 0
                state = State.HOME


def main(args=None):
    rclpy.init(args=args)
    node = CubePointerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
