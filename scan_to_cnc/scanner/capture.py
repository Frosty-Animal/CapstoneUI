"""
stage 1: depth capture via intel realsense d405
supports two modes against a single long-lived pipeline:
  - live preview: light filters, single frame, pv.PolyData for the UI viewport
  - scan capture: heavy filters + temporal averaging, o3d.PointCloud for
    downstream processing and .ply export

the pipeline stays open between calls so both modes can coexist during a
scan (live preview while the carriage moves, averaged capture at each
stop point).
"""

import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import pyvista as pv
import time
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


class RealSenseCapture:
    """Long-lived D405 capture session.

    Thread safety: NOT thread-safe. The rs.pipeline and the filter objects
    hold internal state that breaks if touched from multiple threads. If
    you need to drive this from a QThread and also from scan logic, funnel
    everything through the QThread's command queue — don't call methods
    from the GUI thread directly.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        temporal_frames: int = 15,
        decimation_magnitude: int = 2,
        clip_min_m: float = 0.04,
        clip_max_m: float = 0.50,
        bag_file: str = None,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.temporal_frames = temporal_frames
        self.decimation_magnitude = decimation_magnitude
        self.clip_min_m = clip_min_m
        self.clip_max_m = clip_max_m
        self.bag_file = bag_file

        self.pipeline = None
        self._running = False

        # Two separate filter chains — temporal filter carries state across
        # frames, so live preview and averaged capture MUST NOT share one
        # or they corrupt each other's rolling state.
        self._live_decimation = None
        self._live_spatial = None
        self._live_hole_fill = None
        self._avg_decimation = None
        self._avg_spatial = None
        self._avg_temporal = None
        self._avg_hole_fill = None

        self._pc = None  # rs.pointcloud helper, reused for both paths

    # ── Lifecycle ────────────────────────────────────────────────────────────
    def is_running(self) -> bool:
        return self._running

    def start(self):
        """Open the pipeline and build both filter chains. Idempotent."""
        if self._running:
            return

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if self.bag_file:
            logger.info(f"configuring playback from: {self.bag_file}")
            cfg.enable_device_from_file(self.bag_file, repeat_playback=False)
        else:
            cfg.enable_stream(rs.stream.depth, self.width, self.height,
                              rs.format.z16, self.fps)
            cfg.enable_stream(rs.stream.color, self.width, self.height,
                              rs.format.bgr8, self.fps)

        self.pipeline.start(cfg)
        self._build_filter_chains()
        self._pc = rs.pointcloud()
        self._running = True
        logger.info(f"d405 pipeline up @ {self.width}x{self.height}/{self.fps}fps")

    def stop(self):
        if not self._running:
            return
        try:
            self.pipeline.stop()
        except Exception as e:
            logger.warning(f"pipeline stop raised: {e}")
        self.pipeline = None
        self._pc = None
        self._running = False
        logger.info("realsense pipeline stopped")

    # ── Filter construction ──────────────────────────────────────────────────
    def _build_filter_chains(self):
        # Light chain for live preview: no temporal (we want fresh frames)
        self._live_decimation = rs.decimation_filter()
        self._live_decimation.set_option(rs.option.filter_magnitude,
                                         self.decimation_magnitude)
        self._live_spatial = rs.spatial_filter()
        self._live_spatial.set_option(rs.option.filter_magnitude, 2)
        self._live_spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self._live_spatial.set_option(rs.option.filter_smooth_delta, 20)
        self._live_hole_fill = rs.hole_filling_filter()

        # Heavy chain for averaged capture: includes temporal smoothing
        self._avg_decimation = rs.decimation_filter()
        self._avg_decimation.set_option(rs.option.filter_magnitude,
                                        self.decimation_magnitude)
        self._avg_spatial = rs.spatial_filter()
        self._avg_spatial.set_option(rs.option.filter_magnitude, 2)
        self._avg_spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self._avg_spatial.set_option(rs.option.filter_smooth_delta, 20)
        self._avg_temporal = rs.temporal_filter()
        self._avg_temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
        self._avg_temporal.set_option(rs.option.filter_smooth_delta, 20)
        self._avg_hole_fill = rs.hole_filling_filter()

    def _apply_live_filters(self, depth_frame):
        f = self._live_decimation.process(depth_frame)
        f = self._live_spatial.process(f)
        f = self._live_hole_fill.process(f)
        return f

    def _apply_avg_filters(self, depth_frame):
        f = self._avg_decimation.process(depth_frame)
        f = self._avg_spatial.process(f)
        f = self._avg_temporal.process(f)
        f = self._avg_hole_fill.process(f)
        return f

    # ── Live preview path (UI viewport) ──────────────────────────────────────
    def get_live_polydata(self, timeout_ms: int = 1000) -> pv.PolyData | None:
        """Pull one frame, apply the light filter chain, return pv.PolyData.

        Returns None on timeout or if the frame has no valid points after
        clipping (camera staring at nothing in range).
        """
        if not self._running:
            raise RuntimeError("pipeline not started — call start() first")

        frames = self.pipeline.wait_for_frames(timeout_ms=timeout_ms)
        depth = frames.get_depth_frame()
        if not depth:
            return None

        filtered = self._apply_live_filters(depth)
        points = self._pc.calculate(filtered)
        verts = (np.asanyarray(points.get_vertices())
                   .view(np.float32)
                   .reshape(-1, 3))

        # Clip to D405's trustworthy range
        z = verts[:, 2]
        mask = (z > self.clip_min_m) & (z < self.clip_max_m)
        verts = verts[mask]
        if verts.shape[0] == 0:
            return None

        cloud = pv.PolyData(verts)
        cloud["depth"] = verts[:, 2].copy()
        return cloud

    # ── Scan capture path (averaged, saved to .ply) ──────────────────────────
    def capture_averaged_frames(self):
        """Pull self.temporal_frames frames through the heavy chain and
        return the final (depth, color) pair. The temporal filter's
        internal state does the averaging across frames.
        """
        if not self._running:
            raise RuntimeError("pipeline not started — call start() first")

        depth_frame = None
        color_frame = None
        for _ in range(self.temporal_frames):
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if not d or not c:
                continue
            depth_frame = self._apply_avg_filters(d)
            color_frame = c
        if depth_frame is None:
            raise RuntimeError("no valid depth frames during averaging window")
        return depth_frame, color_frame

    def frames_to_open3d_cloud(self, depth_frame, color_frame) -> o3d.geometry.PointCloud:
        """Build a textured open3d PointCloud from an averaged (depth, color)
        pair. Uses the SDK's pointcloud + texture-coord mapping so intrinsics
        are handled correctly after decimation.
        """
        self._pc.map_to(color_frame)
        points = self._pc.calculate(depth_frame)

        vertices = (np.asanyarray(points.get_vertices())
                      .view(np.float32)
                      .reshape(-1, 3))
        tex_coords = (np.asanyarray(points.get_texture_coordinates())
                        .view(np.float32)
                        .reshape(-1, 2))

        color_h, color_w = self.height, self.width
        u = np.clip((tex_coords[:, 0] * color_w).astype(int), 0, color_w - 1)
        v = np.clip((tex_coords[:, 1] * color_h).astype(int), 0, color_h - 1)
        color_image = np.asanyarray(color_frame.get_data())
        mapped_colors = color_image[v, u][:, ::-1]  # bgr -> rgb

        mask = ~np.all(vertices == 0, axis=1)
        vertices = vertices[mask]
        mapped_colors = mapped_colors[mask]

        logger.info(f"raw point cloud: {vertices.shape[0]} points")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vertices)
        pcd.colors = o3d.utility.Vector3dVector(mapped_colors.astype(np.float64) / 255.0)
        return pcd

    def capture(self, output_path: str = None) -> o3d.geometry.PointCloud:
        """Full averaged single-view capture. Optionally saves to .ply."""
        depth_frame, color_frame = self.capture_averaged_frames()
        pcd = self.frames_to_open3d_cloud(depth_frame, color_frame)
        if output_path:
            o3d.io.write_point_cloud(output_path, pcd)
            logger.info(f"saved point cloud to {output_path}")
        return pcd

    # ── .bag recording (unchanged — uses its own ephemeral pipeline) ─────────
    def record_bag(self, output_bag: str, duration_sec: float = 5.0):
        """Record raw frames to a .bag file. Opens a separate pipeline —
        don't call this while the main pipeline is running.
        """
        if self._running:
            raise RuntimeError("stop() the live pipeline before recording a bag")

        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, self.width, self.height,
                          rs.format.z16, self.fps)
        cfg.enable_stream(rs.stream.color, self.width, self.height,
                          rs.format.bgr8, self.fps)
        cfg.enable_record_to_file(output_bag)

        pipe = rs.pipeline()
        logger.info(f"recording to {output_bag} for {duration_sec}s...")
        pipe.start(cfg)
        start_t = time.time()
        frame_count = 0
        try:
            while time.time() - start_t < duration_sec:
                pipe.wait_for_frames()
                frame_count += 1
        finally:
            pipe.stop()
        logger.info(f"recorded {frame_count} frames to {output_bag}")
