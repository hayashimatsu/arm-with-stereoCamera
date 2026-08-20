"""D455 snapshot capture skeleton for the NHC12 demo scene.

This is a rebuild from scratch, ported from the structure of the reference
implementation (``/home/rci05/User/Lin/test_claude_mcp_04/scripts/capture_d455.py``,
read-only, not this project's earlier ``legacy/capture_d455_m5.py``).

Scope so far (T004 skeleton, T005 async + metric fix): setup, capture RGB +
depth, save to disk, hide the IK target for the frame. No depth-region
segmentation, no self/environment exclusion, no red-cube detection, no
center-of-mass estimate, no annotated overlays, and no
``measurements.json``/``summary.json`` -- see ``docs/CAPTURE_REBUILD_PLAN.md``
for the staged plan that adds those later.

Waiting is all async (T005, B1 -- see docs/CAPTURE_REBUILD_PLAN.md).
``rep.orchestrator.step()`` always raises ``OrchestratorError`` inside a full
Kit GUI session (the exception message itself says "Please use the async
function `step_async`"), so the synchronous version of this module fell back
to a plain update-loop pump on every single call -- it never actually held
Replicator's render-sync guarantee, it was just pumping frames blindly.
``step_async()`` gets that guarantee back. (Script Editor running scripts
inside its own async task is part of the background here too -- see
docs/CAPTURE_REBUILD_PLAN.md -- but on its own it isn't sufficient to explain
T004's two IndexErrors: docs/T004_HIDE_AUDIT.md and reviews/T004.md judgment A
found a same-log case where a synchronous pump under that same async task did
not reproduce it, so treat this as a contributing condition, not the proven
root cause.)

Public entry points:

    demo_capture_setup()   # await; create and cache the three camera render
                            # products. Normally not called directly --
                            # demo_capture_async() awaits it itself.
    demo_capture(label=None)       # GUI-safe: schedules one capture, returns
                                    # {"status": "scheduled"} immediately.
    demo_capture_status()          # result of the most recent demo_capture();
                                    # {"status": "capturing"} while it runs.
    demo_capture_async(label=None) # await; the actual capture. Writes
                                    # rgb_left/right/color.png,
                                    # depth_axial_left.npy,
                                    # depth_radial_left.npy, depth_preview.png,
                                    # result.json.
"""

import asyncio
import builtins
import datetime as _datetime
import hashlib
import json
import math
import os
import re
import traceback

import numpy as np


MOUNT_PATH = (
    "/World/ArmWithHandOnly/Robotiq_2F_85/Robotiq_2F_85/base_link/d455_camera"
)
CAMERA_PATHS = {
    "left": MOUNT_PATH + "/RSD455/Camera_OmniVision_OV9782_Left",
    "right": MOUNT_PATH + "/RSD455/Camera_OmniVision_OV9782_Right",
    "color": MOUNT_PATH + "/RSD455/Camera_OmniVision_OV9782_Color",
}
IK_TARGET_PATH = "/World/IKTarget"
# docs/TOPOLOGY.md. Verified present (T006 Step 5, live-stage check, prim
# type Xform) before this constant was written -- do not guess another path
# if it's ever missing; stop and report instead.
ARM_BASE_PATH = "/World/ArmWithHandOnly/NHC12_A00/base_link"
WIDTH = 640
HEIGHT = 480
REGISTRY_NAME = "_nhc12_d455_capture_registry"


def _registry():
    if not hasattr(builtins, REGISTRY_NAME):
        setattr(
            builtins,
            REGISTRY_NAME,
            {
                "stage_id": None,
                "render_products": None,
                "annotators": None,
                "capture_task": None,
                "last_capture_result": None,
            },
        )
    return getattr(builtins, REGISTRY_NAME)


def _active_stage_context():
    import omni.usd

    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")
    layer = stage.GetRootLayer()
    stage_path = layer.realPath or layer.identifier
    if not stage_path or not os.path.isfile(stage_path):
        raise RuntimeError(
            "The active stage must be a saved local USD before demo capture."
        )
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(stage_path)))
    output_root = os.environ.get(
        "D455_DEMO_OUTPUT_DIR", os.path.join(project_root, "outputs", "captures")
    )
    return context, stage, os.path.abspath(stage_path), os.path.abspath(output_root)


def _sanitize_label(label):
    if label is None:
        return None
    value = re.sub(r"[^A-Za-z0-9._-]+", "-", str(label).strip()).strip("-._")
    return value[:48] or None


def _create_unique_run_dir(output_root, label=None):
    os.makedirs(output_root, exist_ok=True)
    now_utc = _datetime.datetime.now(_datetime.timezone.utc)
    timestamp = now_utc.strftime("%Y%m%dT%H%M%S%fZ")
    date_prefix = now_utc.astimezone().strftime("%Y-%m%d")
    suffix = _sanitize_label(label)
    existing_pattern = re.compile(
        rf"^{re.escape(date_prefix)}-(\d+)(?:-[A-Za-z0-9._-]+)?$"
    )
    existing_numbers = []
    for name in os.listdir(output_root):
        match = existing_pattern.match(name)
        if match and os.path.isdir(os.path.join(output_root, name)):
            existing_numbers.append(int(match.group(1)))
    first_sequence = max(existing_numbers, default=0) + 1
    for sequence in range(first_sequence, first_sequence + 1000):
        base = f"{date_prefix}-{sequence}"
        run_id = base if suffix is None else f"{base}-{suffix}"
        run_dir = os.path.join(output_root, run_id)
        try:
            os.makedirs(run_dir)
            return run_id, run_dir, timestamp, suffix, sequence
        except FileExistsError:
            continue
    raise RuntimeError("Could not allocate a unique capture directory.")


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _source_file_path():
    """Best-effort path to this file on disk, independent of __file__ --
    exec(code, globals()) loading from demo_start.py never sets __file__,
    which is exactly the gap T006 Step 7b exists to catch (a stale exec'd
    demo_capture in Script Editor globals producing pre-T006 output long
    after the file on disk moved on -- the M6 Finding 3 contamination
    pattern repeating). Falls back to the stage's own project root. Never
    raises; returns None if neither works."""
    path = globals().get("__file__")
    if path and os.path.isfile(path):
        return path
    try:
        _, _, stage_path, _ = _active_stage_context()
        candidate = os.path.join(os.path.dirname(os.path.dirname(stage_path)), "scripts", "capture_d455.py")
        return candidate if os.path.isfile(candidate) else None
    except Exception:
        return None


def _source_sha256_or_none():
    path = _source_file_path()
    try:
        return _sha256(path) if path else None
    except Exception:
        return None


def _loaded_source_sha256_at_bootstrap():
    """T008 Step 0: T006 Step 7b's original staleness baseline
    (SOURCE_SHA256_AT_LOAD, computed at this module's own exec time) was
    structurally unable to ever read stale=true once callers started
    re-exec'ing this file on every call -- the "loaded" hash and the "disk"
    hash were always the same exec. The real question is "did the file on
    disk change since demo_start.py's bootstrap exec", so the baseline now
    comes from a key demo_start.py writes into this same builtins registry
    right before it execs this file. None if that key isn't there (e.g.
    this module was exec'd some other way) -- reported as unknown, not
    coerced to a false pass."""
    return _registry().get("capture_source_sha256_at_bootstrap")


def _camera_frame(stage, camera_path):
    import omni.timeline

    prim = stage.GetPrimAtPath(camera_path)
    if not prim.IsValid():
        raise RuntimeError(f"Camera prim not found: {camera_path}")
    if omni.timeline.get_timeline_interface().is_playing():
        from isaacsim.core.experimental.prims import XformPrim

        positions, quaternions = XformPrim(camera_path).get_world_poses()
        position = positions.numpy()[0].astype(np.float64)
        w, x, y, z = quaternions.numpy()[0].astype(np.float64)
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        right, up, back = rotation[:, 0], rotation[:, 1], rotation[:, 2]
    else:
        from pxr import Usd, UsdGeom

        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        position = np.asarray(matrix.ExtractTranslation(), dtype=np.float64)
        authored_rotation = matrix.ExtractRotationMatrix()
        axes = np.array(
            [
                [authored_rotation[row][column] for column in range(3)]
                for row in range(3)
            ],
            dtype=np.float64,
        )
        right, up, back = axes[0], axes[1], axes[2]
    right = right / np.linalg.norm(right)
    up = up / np.linalg.norm(up)
    forward = -back / np.linalg.norm(back)
    return {
        "position_m": position.tolist(),
        "right": right.tolist(),
        "up": up.tolist(),
        "forward": forward.tolist(),
    }


def _arm_base_world_xyz(stage):
    """World position of ARM_BASE_PATH. Same runtime-vs-authored rule as
    _camera_frame() (AGENTS.md: read runtime/Fabric state while Play is
    active, not authored values) -- the arm base itself doesn't move, but
    this keeps the two position reads consistent."""
    import omni.timeline

    prim = stage.GetPrimAtPath(ARM_BASE_PATH)
    if not prim.IsValid():
        raise RuntimeError(f"Arm base prim not found: {ARM_BASE_PATH}")
    if omni.timeline.get_timeline_interface().is_playing():
        from isaacsim.core.experimental.prims import XformPrim

        positions, _ = XformPrim(ARM_BASE_PATH).get_world_poses()
        position = positions.numpy()[0].astype(np.float64)
    else:
        from pxr import Usd, UsdGeom

        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        position = np.asarray(matrix.ExtractTranslation(), dtype=np.float64)
    return position.tolist()


# --- T006 measurement pure functions -------------------------------------
# No omni imports, no await, no app.update(). Take numpy arrays and plain
# numbers/dicts, return plain numbers/dicts. This is what keeps
# demo_measure_capture() in legacy/capture_d455_m5.py's 1420-line measurement
# layer at zero event-loop calls -- do the same here from the start
# (docs/CAPTURE_REBUILD_PLAN.md).


# SELF_BODY_MAX_CAMERA_DISTANCE_M is not a free parameter -- it comes from
# legacy/capture_d455_m5.py's SELF_BODY_MAX_CAMERA_DISTANCE_M (measured
# Robotiq 2F-85 AABB envelope, 0.167 m from the camera at its furthest) and
# this task's own diagnosis of run 2026-0820-3: depth is completely empty
# from 0.18-0.45 m (self-body/scene physical gap), so 0.180 m sits safely in
# that gap without needing recalibration. Do not change this value.
SELF_BODY_MAX_CAMERA_DISTANCE_M = 0.180


def _select_target_surface_pixel(depth_axial, self_body_max_m=SELF_BODY_MAX_CAMERA_DISTANCE_M):
    """(u, v) of the nearest valid depth pixel beyond the self-body envelope.

    No object identification -- just the nearest thing depth saw that isn't
    the robot's own gripper. Returns target_pixel_px=None (not a fallback to
    a self-body pixel) if nothing valid remains past the exclusion.
    """
    valid = np.isfinite(depth_axial) & (depth_axial > 0)
    excluded = valid & (depth_axial < self_body_max_m)
    remaining = valid & ~excluded
    remaining_depths = depth_axial[remaining]
    excluded_depths = depth_axial[excluded]
    stats = {
        "target_pixel_px": None,
        "self_body_excluded_px": int(np.count_nonzero(excluded)),
        "self_body_max_m": self_body_max_m,
        "valid_px": int(np.count_nonzero(valid)),
        "remaining_px": int(np.count_nonzero(remaining)),
        "nearest_remaining_depth_m": float(remaining_depths.min()) if remaining_depths.size else None,
        "nearest_excluded_depth_m": float(excluded_depths.min()) if excluded_depths.size else None,
    }
    if remaining_depths.size:
        masked = np.where(remaining, depth_axial, np.inf)
        v, u = np.unravel_index(np.argmin(masked), masked.shape)
        stats["target_pixel_px"] = [int(u), int(v)]
    return stats


def _intrinsics_from_fov(width, height, horizontal_fov_rad):
    """Geometric pinhole intrinsics from image size and horizontal FOV.

    Square pixels assumed (fx == fy); no calibration data used, see
    tasks/T006-basic-measurement/TASK.md Step 1 (T007+ territory otherwise).
    """
    fx = (width / 2.0) / math.tan(horizontal_fov_rad / 2.0)
    return {"fx": fx, "fy": fx, "cx": width / 2.0, "cy": height / 2.0, "source": "geometric"}


def _pixel_depth_to_camera_xyz(u, v, depth_m, intrinsics):
    """Back-project one pixel + its axial depth into camera-frame [right, up, forward].

    Matches _camera_frame()'s right/up/forward basis: forward is the optical
    axis (what axial depth measures along), u grows rightward (same sign as
    right), v grows downward (opposite sign from up).
    """
    x_right = (u - intrinsics["cx"]) * depth_m / intrinsics["fx"]
    y_up = -(v - intrinsics["cy"]) * depth_m / intrinsics["fy"]
    return [x_right, y_up, float(depth_m)]


def _camera_to_world_xyz(camera_xyz, camera_frame):
    """camera_frame is one entry of _camera_frame()'s return dict."""
    position = np.asarray(camera_frame["position_m"], dtype=np.float64)
    right = np.asarray(camera_frame["right"], dtype=np.float64)
    up = np.asarray(camera_frame["up"], dtype=np.float64)
    forward = np.asarray(camera_frame["forward"], dtype=np.float64)
    x_right, y_up, z_forward = camera_xyz
    world = position + x_right * right + y_up * up + z_forward * forward
    return world.tolist()


def _world_xyz_to_camera_xyz(world_xyz, camera_frame):
    """Inverse of _camera_to_world_xyz() -- world point into [right, up, forward]."""
    position = np.asarray(camera_frame["position_m"], dtype=np.float64)
    vector = np.asarray(world_xyz, dtype=np.float64) - position
    right = np.asarray(camera_frame["right"], dtype=np.float64)
    up = np.asarray(camera_frame["up"], dtype=np.float64)
    forward = np.asarray(camera_frame["forward"], dtype=np.float64)
    return [float(np.dot(vector, right)), float(np.dot(vector, up)), float(np.dot(vector, forward))]


def _camera_xyz_to_pixel(camera_xyz, intrinsics):
    """Inverse of _pixel_depth_to_camera_xyz(). None if behind the camera
    (z_forward <= 0) -- the caller decides what "outside FOV" means."""
    x_right, y_up, z_forward = camera_xyz
    if z_forward <= 0:
        return None
    u = intrinsics["cx"] + x_right * intrinsics["fx"] / z_forward
    v = intrinsics["cy"] - y_up * intrinsics["fy"] / z_forward
    return u, v


def _collect_failures(rgb_diagnostics, settle, green_audit):
    """Pure: this project's one quality gate. Do not loosen green_audit's
    two conditions -- see docs/CAPTURE_REBUILD_PLAN.md and reviews/T004.md
    judgment B (total_px catches signal center_px alone would miss)."""
    failures = [
        f"{eye} RGB is black or invalid"
        for eye, diagnostic in rgb_diagnostics.items()
        if not diagnostic["valid"]
    ]
    if not settle.get("settled", False):
        failures.append("IK target did not converge before capture timeout")
    for eye, audit in green_audit.items():
        if audit["center_px"] > 100:
            failures.append(f"IK target visible near image centre in {eye} RGB")
        if audit["total_px"] > 100:
            failures.append(f"unexplained green pixels in {eye} RGB")
    return failures


def _displacement(from_xyz, to_xyz):
    """World-frame vector from_xyz -> to_xyz, and its norm."""
    vector = np.asarray(to_xyz, dtype=np.float64) - np.asarray(from_xyz, dtype=np.float64)
    return vector.tolist(), float(np.linalg.norm(vector))


def _round_floats(value, ndigits=6):
    """Recursively round floats in a JSON-shaped value -- readability only,
    not a precision claim (see result.json's accuracy_note)."""
    if isinstance(value, float):
        return round(value, ndigits)
    if isinstance(value, dict):
        return {key: _round_floats(item, ndigits) for key, item in value.items()}
    if isinstance(value, list):
        return [_round_floats(item, ndigits) for item in value]
    return value


# --- end T006 measurement pure functions ----------------------------------


# --- T007 tabletop object pure functions ----------------------------------
# Same rule as T006: no omni imports, no await, offline-runnable on saved
# .npy/.png/.json. tasks/T007-target-identification/TASK.md "planner 的方法
# 評估" is the spec these implement; see that section for the why.

BORDER_MARGIN_PX = 2  # tasks/T007.../TASK.md Step 2, ported from legacy/capture_d455_m5.py
_PLANE_TOLERANCE_M = 0.005  # RANSAC inlier tolerance; also region_overlay's TABLE band


def _backproject_depth_to_world(depth_axial, intrinsics, camera_frame):
    """Vectorized form of _pixel_depth_to_camera_xyz + _camera_to_world_xyz
    over every pixel at once -- same formula, cross-checked against the
    scalar functions in the offline T007 harness. Returns (H, W, 3); values
    at invalid pixels are meaningless and must be masked by the caller."""
    height, width = depth_axial.shape
    v_idx, u_idx = np.mgrid[0:height, 0:width]
    depth = depth_axial.astype(np.float64)
    x_right = (u_idx - intrinsics["cx"]) * depth / intrinsics["fx"]
    y_up = -(v_idx - intrinsics["cy"]) * depth / intrinsics["fy"]
    position = np.asarray(camera_frame["position_m"], dtype=np.float64)
    right = np.asarray(camera_frame["right"], dtype=np.float64)
    up = np.asarray(camera_frame["up"], dtype=np.float64)
    forward = np.asarray(camera_frame["forward"], dtype=np.float64)
    return (
        position
        + x_right[..., None] * right
        + y_up[..., None] * up
        + depth[..., None] * forward
    )


def _fit_dominant_plane(points_world, iterations=200, tolerance_m=_PLANE_TOLERANCE_M, seed=0):
    """RANSAC plane fit over a world-frame point cloud, refined by SVD over
    the winning inlier set. Normal is oriented toward +Z (tabletop faces up).
    None if fewer than 3 points are given."""
    points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    count = points.shape[0]
    if count < 3:
        return None
    rng = np.random.default_rng(seed)
    best_inliers, best_count = None, -1
    for _ in range(iterations):
        p0, p1, p2 = points[rng.choice(count, size=3, replace=False)]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        offset = float(np.dot(normal, p0))
        inliers = np.abs(points @ normal - offset) <= tolerance_m
        inlier_count = int(np.count_nonzero(inliers))
        if inlier_count > best_count:
            best_inliers, best_count = inliers, inlier_count
    if best_inliers is None or best_count < 3:
        return None
    inlier_points = points[best_inliers]
    centroid = inlier_points.mean(axis=0)
    _, _, vt = np.linalg.svd(inlier_points - centroid, full_matrices=False)
    normal = vt[-1] / np.linalg.norm(vt[-1])
    if normal[2] < 0:
        normal = -normal
    return {
        "normal": normal.tolist(),
        "offset_m": float(np.dot(normal, centroid)),
        "inlier_px": best_count,
        "inlier_fraction": float(best_count / count),
    }


def _mask_from_rows_cols(rows, cols, shape):
    mask = np.zeros(shape, dtype=bool)
    mask[rows, cols] = True
    return mask


def _tabletop_object_regions(
    depth_axial,
    points_world,
    plane,
    intrinsics,
    self_body_max_m=SELF_BODY_MAX_CAMERA_DISTANCE_M,
    above_plane_min_m=0.010,
    above_plane_max_m=0.500,
    min_px=80,
    depth_step_m=0.02,
):
    """Segment pixels above the table plane into 8-connected regions (depth
    step > depth_step_m breaks connectivity). intrinsics is accepted for
    signature parity with TASK.md's spec but unused here -- points_world is
    already back-projected, so no pixel->camera re-derivation is needed."""
    del intrinsics
    height, width = depth_axial.shape
    valid = np.isfinite(depth_axial) & (depth_axial > 0)
    self_body_mask = valid & (depth_axial < self_body_max_m)
    normal = np.asarray(plane["normal"], dtype=np.float64)
    signed_height = np.dot(points_world, normal) - plane["offset_m"]
    above_mask = (
        valid
        & ~self_body_mask
        & (signed_height >= above_plane_min_m)
        & (signed_height <= above_plane_max_m)
    )
    visited = np.zeros((height, width), dtype=bool)
    regions = []
    for start_v in range(height):
        for start_u in range(width):
            if not above_mask[start_v, start_u] or visited[start_v, start_u]:
                continue
            stack = [(start_v, start_u)]
            visited[start_v, start_u] = True
            rows, cols = [], []
            while stack:
                v, u = stack.pop()
                rows.append(v)
                cols.append(u)
                current_depth = float(depth_axial[v, u])
                for dv in (-1, 0, 1):
                    for du in (-1, 0, 1):
                        nv, nu = v + dv, u + du
                        if dv == 0 and du == 0:
                            continue
                        if not (0 <= nv < height and 0 <= nu < width):
                            continue
                        if not above_mask[nv, nu] or visited[nv, nu]:
                            continue
                        if abs(float(depth_axial[nv, nu]) - current_depth) <= depth_step_m:
                            visited[nv, nu] = True
                            stack.append((nv, nu))
            if len(rows) < min_px:
                continue
            rows_arr, cols_arr = np.asarray(rows), np.asarray(cols)
            bbox = [int(cols_arr.min()), int(rows_arr.min()), int(cols_arr.max()), int(rows_arr.max())]
            touches_border = bool(
                bbox[0] <= BORDER_MARGIN_PX
                or bbox[1] <= BORDER_MARGIN_PX
                or bbox[2] >= width - 1 - BORDER_MARGIN_PX
                or bbox[3] >= height - 1 - BORDER_MARGIN_PX
            )
            regions.append(
                {
                    "label": len(regions) + 1,
                    "pixel_count": int(len(rows)),
                    "bbox_px": bbox,
                    "height_above_plane_m": float(np.median(signed_height[rows_arr, cols_arr])),
                    "depth_median_m": float(np.median(depth_axial[rows_arr, cols_arr])),
                    "touches_border": touches_border,
                    "mask_rows": rows_arr,
                    "mask_cols": cols_arr,
                }
            )
    return regions


def _region_surface_center(region_mask, points_world, depth_axial):
    """tasks/T007.../TASK.md 'planner 的方法評估 -> 表面中心怎麼算', steps 1-5."""
    del depth_axial  # kept in the signature for TASK.md parity; unused here
    ys, xs = np.nonzero(region_mask)
    points = points_world[ys, xs]
    centroid = points.mean(axis=0)
    distances = np.linalg.norm(points - centroid, axis=1)
    best = int(np.argmin(distances))
    surface_center = points[best]
    extent = points.max(axis=0) - points.min(axis=0)
    return {
        "surface_centroid_world_xyz_m": centroid.tolist(),
        "representative_pixel_px": [int(xs[best]), int(ys[best])],
        "surface_center_world_xyz_m": surface_center.tolist(),
        "centroid_to_surface_offset_m": float(np.linalg.norm(centroid - surface_center)),
        "surface_extent_world_m": {
            "x": float(extent[0]),
            "y": float(extent[1]),
            "z": float(extent[2]),
        },
    }


def _rgb_to_hsv(rgb):
    """Vectorized RGB -> HSV: hue in degrees [0, 360), saturation/value on
    the 0-1 scale."""
    array = rgb.astype(np.float64) / 255.0
    r, g, b = array[..., 0], array[..., 1], array[..., 2]
    cmax = array.max(axis=-1)
    cmin = array.min(axis=-1)
    delta = cmax - cmin
    safe_delta = np.where(delta == 0, 1.0, delta)
    is_r = (cmax == r) & (delta != 0)
    is_g = (cmax == g) & (delta != 0) & ~is_r
    is_b = (cmax == b) & (delta != 0) & ~is_r & ~is_g
    hue = np.zeros_like(cmax)
    hue = np.where(is_r, 60.0 * (((g - b) / safe_delta) % 6.0), hue)
    hue = np.where(is_g, 60.0 * (((b - r) / safe_delta) + 2.0), hue)
    hue = np.where(is_b, 60.0 * (((r - g) / safe_delta) + 4.0), hue)
    saturation = np.where(cmax == 0, 0.0, delta / np.where(cmax == 0, 1.0, cmax))
    return hue, saturation, cmax


# T008 Step 0b: reviews/T007.md found the basket's own dark, low-saturation
# green material crossing the hue-only >0.5 gate (0.402/0.493/0.257 across 3
# runs, one within 0.007 of the threshold) -- a false-positive mode, not an
# unhidden IK target. The IK target is both high-saturation and bright;
# gating on S/V too is how the two are told apart. Thresholds are the
# planner's values from reviews/T007.md's decision, not tuned by the coder.
_GREEN_SATURATION_MIN = 0.45
_GREEN_VALUE_MIN = 0.35


def _region_green_fraction(rgb_left, region_mask):
    """Returns (green_fraction, green_hue_only_fraction). green_fraction
    requires hue in [90,150] deg AND saturation >= 0.45 AND value >= 0.35;
    green_hue_only_fraction is the old hue-only definition, kept for
    comparison (T008 Step 0b)."""
    hue, sat, val = _rgb_to_hsv(rgb_left)
    hue, sat, val = hue[region_mask], sat[region_mask], val[region_mask]
    if hue.size == 0:
        return 0.0, 0.0
    hue_only = (hue >= 90.0) & (hue <= 150.0)
    green = hue_only & (sat >= _GREEN_SATURATION_MIN) & (val >= _GREEN_VALUE_MIN)
    return (
        float(np.count_nonzero(green) / hue.size),
        float(np.count_nonzero(hue_only) / hue.size),
    )


def _build_tabletop_objects(
    depth_axial,
    rgb_left,
    intrinsics,
    camera_frame,
    arm_base_world_xyz_m,
    self_body_max_m=SELF_BODY_MAX_CAMERA_DISTANCE_M,
):
    """Composes Steps 1-4: plane fit -> region segmentation -> per-region
    surface center + green check. Pure; the only T007 entry point
    demo_capture_async() calls."""
    points_world = _backproject_depth_to_world(depth_axial, intrinsics, camera_frame)
    valid = np.isfinite(depth_axial) & (depth_axial > 0)
    self_body_mask = valid & (depth_axial < self_body_max_m)
    plane = _fit_dominant_plane(points_world[valid & ~self_body_mask])
    if plane is None:
        return {
            "plane": None, "objects": [], "failures": ["table plane fit failed -- insufficient points"],
            "table_mask": np.zeros(depth_axial.shape, dtype=bool), "points_world": points_world,
        }
    normal = np.asarray(plane["normal"], dtype=np.float64)
    signed_height = np.dot(points_world, normal) - plane["offset_m"]
    table_mask = valid & ~self_body_mask & (np.abs(signed_height) <= _PLANE_TOLERANCE_M)
    regions = _tabletop_object_regions(depth_axial, points_world, plane, intrinsics, self_body_max_m=self_body_max_m)
    objects, failures = [], []
    for region in regions:
        mask = _mask_from_rows_cols(region["mask_rows"], region["mask_cols"], depth_axial.shape)
        center = _region_surface_center(mask, points_world, depth_axial)
        green_fraction, green_hue_only_fraction = _region_green_fraction(rgb_left, mask)
        if green_fraction > 0.5:
            failures.append(f"detected object {region['label']} is dominated by green hue -- IK target may not be hidden")
        cam_vec, cam_norm = _displacement(camera_frame["position_m"], center["surface_center_world_xyz_m"])
        arm_vec, _ = _displacement(arm_base_world_xyz_m, center["surface_center_world_xyz_m"])
        objects.append(
            {
                "id": region["label"],
                "pixel_count": region["pixel_count"],
                "bbox_px": region["bbox_px"],
                "height_above_plane_m": region["height_above_plane_m"],
                "depth_median_m": region["depth_median_m"],
                "touches_border": region["touches_border"],
                "center_reliability": "truncated_by_image_border" if region["touches_border"] else "ok",
                "camera_to_surface_center_distance_m": cam_norm,
                "vector_camera_to_object_m": cam_vec,
                "vector_armbase_to_object_m": arm_vec,
                "green_hue_fraction": green_fraction,
                "green_hue_only_fraction": green_hue_only_fraction,
                "mask": mask,
                **center,
            }
        )
    return {"plane": plane, "objects": objects, "failures": failures, "table_mask": table_mask, "points_world": points_world}


_TABLETOP_JSON_KEYS = (
    "id", "pixel_count", "bbox_px", "height_above_plane_m", "depth_median_m",
    "representative_pixel_px", "surface_center_world_xyz_m",
    "surface_centroid_world_xyz_m", "centroid_to_surface_offset_m",
    "surface_extent_world_m", "touches_border", "center_reliability",
    "camera_to_surface_center_distance_m", "vector_camera_to_object_m",
    "vector_armbase_to_object_m", "green_hue_fraction", "green_hue_only_fraction",
)


def _build_tabletop_objects_json(tabletop):
    """tabletop_objects section of result.json (T007 Step 5). Drops the
    internal-only 'mask' array that annotation drawing needs but the JSON
    output must not carry."""
    plane = tabletop["plane"]
    plane_json = None if plane is None else {
        "normal": plane["normal"], "offset_m": plane["offset_m"], "inlier_fraction": plane["inlier_fraction"],
    }
    objects_json = [{key: obj[key] for key in _TABLETOP_JSON_KEYS} for obj in tabletop["objects"]]
    return _round_floats(
        {
            "method": "plane_fit_then_connected_regions_v1",
            "plane": plane_json,
            "params": {
                "self_body_max_m": SELF_BODY_MAX_CAMERA_DISTANCE_M,
                "above_plane_min_m": 0.010,
                "above_plane_max_m": 0.500,
                "min_px": 80,
                "depth_step_m": 0.02,
            },
            "object_count": len(objects_json),
            "objects": objects_json,
            "note": (
                "All tabletop objects above the fitted table plane, excluding the "
                "robot's own body and the table surface itself. No semantic "
                "identification performed. Coordinates are surface centres, not "
                "volumetric centres."
            ),
        }
    )


# --- end T007 tabletop object pure functions -------------------------------


def _save_png(array, path):
    from PIL import Image

    Image.fromarray(array.astype(np.uint8)).save(path)


def _depth_preview(depth):
    valid_mask = np.isfinite(depth) & (depth > 0)
    preview = np.zeros(depth.shape, dtype=np.uint8)
    if np.any(valid_mask):
        values = depth[valid_mask]
        low, high = np.percentile(values, [2.0, 98.0])
        scaled = np.clip((depth - low) / max(float(high - low), 1e-6), 0.0, 1.0)
        preview[valid_mask] = (scaled[valid_mask] * 255.0).astype(np.uint8)
    return np.stack([preview, preview, preview], axis=-1)


async def _force_render_step(rt_subframes=4):
    import omni.kit.app

    try:
        import omni.replicator.core as rep

        await rep.orchestrator.step_async(
            rt_subframes=rt_subframes,
            pause_timeline=False,
        )
        return {"method": "orchestrator_step_async", "exception": None}
    except Exception as exc:
        # step_async() is the documented replacement for the synchronous
        # step() that always raised OrchestratorError inside a full Kit GUI
        # session (see docs/CAPTURE_REBUILD_PLAN.md) -- it should not raise
        # here, but the fallback is kept intentionally so a capture never
        # hard-fails if Replicator's async path changes underneath us. Any
        # exception type falls back the same way, but the type and message
        # are recorded rather than swallowed (reviews/T005.md Step 7: a bare
        # except here made a silent regression to blind pumping impossible
        # to observe after the fact).
        app = omni.kit.app.get_app()
        for _ in range(max(1, rt_subframes)):
            await app.next_update_async()
        return {
            "method": "app_update_fallback",
            "exception": f"{type(exc).__name__}: {exc}",
        }


async def _read_rgb_with_retry(annotator, max_retries=4, minimum_mean=1.0):
    attempts = []
    image = None
    for attempt in range(max_retries + 1):
        image = np.asarray(annotator.get_data())[:, :, :3].astype(np.uint8)
        mean = float(image.mean())
        attempts.append(mean)
        if mean >= minimum_mean:
            return image, {
                "mean_at_each_attempt": attempts,
                "retries": attempt,
                "valid": True,
            }
        await _force_render_step()
    return image, {
        "mean_at_each_attempt": attempts,
        "retries": max_retries,
        "valid": False,
    }


async def _wait_for_ik_settle(max_frames=180, stable_frames=5):
    """Wait for the running IK follow controller to reach its target.

    If no controller is active, only a short render warm-up is performed so a
    static scene can still be captured without IK running.
    """
    import omni.kit.app

    app = omni.kit.app.get_app()
    controller_registry = getattr(
        builtins, "_arm310d_ik_follow_controller_registry", {"instance": None}
    )
    controller = controller_registry.get("instance")
    if controller is None:
        for _ in range(10):
            await app.next_update_async()
        return {"settled": True, "reason": "IK controller is not running"}

    stable = 0
    last_status = None
    for frame in range(1, max_frames + 1):
        await app.next_update_async()
        last_status = controller.status()
        error = last_status.get("last_error")
        if isinstance(error, dict):
            position_ok = error.get("pos_error_norm_m", 999.0) <= 0.003
            orientation_ok = error.get("rot_error_norm_rad", 999.0) <= 0.05
            stable = stable + 1 if position_ok and orientation_ok else 0
            if stable >= stable_frames:
                return {
                    "settled": True,
                    "frames": frame,
                    "controller_status": last_status,
                }
    return {
        "settled": False,
        "frames": max_frames,
        "controller_status": last_status,
        "reason": "IK target did not converge before capture timeout",
    }


async def demo_capture_setup(force=False):
    """Create and cache the three camera render products for the active stage."""
    import omni.kit.app
    import omni.replicator.core as rep
    import omni.timeline

    context, stage, _, _ = _active_stage_context()
    registry = _registry()
    stage_id = context.get_stage_id()
    if (
        not force
        and registry["stage_id"] == stage_id
        and registry["render_products"] is not None
    ):
        return {"status": "already_setup", "stage_id": stage_id}

    render_products = {}
    annotators = {}
    for eye, path in CAMERA_PATHS.items():
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"Camera prim not found: {path}")
        product = rep.create.render_product(path, (WIDTH, HEIGHT))
        render_products[eye] = product
        eye_annotators = {"rgb": rep.AnnotatorRegistry.get_annotator("rgb")}
        eye_annotators["rgb"].attach(product)
        if eye in ("left", "right"):
            for name in ("distance_to_image_plane", "distance_to_camera"):
                eye_annotators[name] = rep.AnnotatorRegistry.get_annotator(name)
                eye_annotators[name].attach(product)
        annotators[eye] = eye_annotators

    timeline = omni.timeline.get_timeline_interface()
    was_playing = timeline.is_playing()
    if not was_playing:
        timeline.play()
    app = omni.kit.app.get_app()
    for _ in range(30):
        await app.next_update_async()
    render_method = await _force_render_step(rt_subframes=8)

    registry.update(
        {
            "stage_id": stage_id,
            "render_products": render_products,
            "annotators": annotators,
        }
    )
    return {
        "status": "setup_complete",
        "stage_id": stage_id,
        "timeline_was_playing": was_playing,
        "render_method": render_method,
    }


# Center region matches tools/green_audit.py's CENTER exactly -- result.json
# (online) and the offline audit tool must always agree on the same numbers.
_GREEN_CENTER_X0, _GREEN_CENTER_X1 = 160, 480
_GREEN_CENTER_Y0, _GREEN_CENTER_Y1 = 120, 360


def _green_audit_eye(rgb):
    red = rgb[:, :, 0].astype(np.int32)
    green = rgb[:, :, 1].astype(np.int32)
    blue = rgb[:, :, 2].astype(np.int32)
    mask = (green > 100) & (green > red * 1.6) & (green > blue * 1.6)
    total_px = int(np.count_nonzero(mask))
    center_px = int(
        np.count_nonzero(
            mask[_GREEN_CENTER_Y0:_GREEN_CENTER_Y1, _GREEN_CENTER_X0:_GREEN_CENTER_X1]
        )
    )
    if total_px == 0:
        return {"total_px": 0, "center_px": 0, "bbox_px": None, "centroid_px": None}
    ys, xs = np.nonzero(mask)
    return {
        "total_px": total_px,
        "center_px": center_px,
        "bbox_px": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
        "centroid_px": [int(xs.mean()), int(ys.mean())],
    }


def _build_result_json(
    run_id,
    failures,
    camera_world_xyz_m,
    arm_base_world_xyz_m,
    measurement,
    images,
    source_integrity,
    target_outside_color_fov,
    tabletop,
):
    """Assembles the user-facing result.json dict (T006 Step 6). Not async,
    touches no omni state -- everything it needs is already plain data."""
    target_world_xyz_m = measurement["target_world_xyz_m"]
    if target_world_xyz_m is None:
        vectors = {
            "vector_camera_to_target_m": None,
            "vector_camera_to_target_norm_m": None,
            "vector_armbase_to_target_m": None,
            "vector_armbase_to_target_norm_m": None,
        }
    else:
        cam_vec, cam_norm = _displacement(camera_world_xyz_m, target_world_xyz_m)
        arm_vec, arm_norm = _displacement(arm_base_world_xyz_m, target_world_xyz_m)
        vectors = {
            "vector_camera_to_target_m": cam_vec,
            "vector_camera_to_target_norm_m": cam_norm,
            "vector_armbase_to_target_m": arm_vec,
            "vector_armbase_to_target_norm_m": arm_norm,
        }
    return _round_floats(
        {
            "status": "pass" if not failures else "fail",
            "run_id": run_id,
            "failures": failures,
            "units": {
                "length": "m",
                "angle": "rad",
                "pixel": "px",
                "note": "fields ending in _m are metres, _cm are centimetres, _px are pixels",
            },
            "accuracy_note": (
                "Simulated RTX depth. No camera calibration performed. Numeric precision "
                "is unverified and must not be treated as a calibrated metrological result."
            ),
            "camera_world_xyz_m": camera_world_xyz_m,
            "arm_base_world_xyz_m": arm_base_world_xyz_m,
            "target_pixel_px": measurement["target_pixel_px"],
            "camera_to_target_distance_m": measurement["camera_to_target_distance_m"],
            "camera_to_target_distance_cm": measurement["camera_to_target_distance_cm"],
            "target_camera_xyz_m": measurement["target_camera_xyz_m"],
            "target_world_xyz_m": target_world_xyz_m,
            **vectors,
            "self_body_exclusion": measurement["self_body_exclusion"],
            "intrinsics": measurement["intrinsics"],
            "images": images,
            "target_outside_color_fov": target_outside_color_fov,
            "source_integrity": source_integrity,
            "target_identification": {
                "method": "nearest_surface_beyond_self_body",
                "note": (
                    "legacy T006 rule: nearest surface beyond self-body; kept for "
                    "comparison"
                ),
            },
            "tabletop_objects": _build_tabletop_objects_json(tabletop),
        }
    )


def _build_diagnostics_json(
    green_audit,
    render,
    frames,
    stage_sha256_before,
    stage_sha256_after,
    timeline_playing_before,
    ik_target_hidden,
    stereo_baseline_m,
    settle,
):
    """Everything moved out of result.json (T006 Step 6) -- the internals a
    human debugging a bad run needs, not what they read day to day."""
    return _round_floats(
        {
            "green_audit": green_audit,
            "render": render,
            "camera_frames": frames,
            "stage_sha256_before": stage_sha256_before,
            "stage_sha256_after": stage_sha256_after,
            "timeline_playing_before": timeline_playing_before,
            "ik_target_hidden": ik_target_hidden,
            "stereo_baseline_m": stereo_baseline_m,
            "settle": settle,
        }
    )


def _save_capture_products(run_dir, rgb, axial_left, radial_left):
    """Write the six saved files for one run. Touches disk, not omni."""
    _save_png(rgb["color"], os.path.join(run_dir, "rgb_color.png"))
    _save_png(rgb["left"], os.path.join(run_dir, "rgb_left.png"))
    _save_png(rgb["right"], os.path.join(run_dir, "rgb_right.png"))
    np.save(os.path.join(run_dir, "depth_axial_left.npy"), axial_left)
    np.save(os.path.join(run_dir, "depth_radial_left.npy"), radial_left)
    _save_png(_depth_preview(axial_left), os.path.join(run_dir, "depth_preview.png"))


# docs/ANNOTATION_SPEC.md fixed palette -- never green, that's green_audit's
# detection colour.
_TARGET_COLOR = (255, 0, 255)
_NEAREST_COLOR = (0, 200, 255)  # reserved by docs/ANNOTATION_SPEC.md; T007's
# Step 6 doesn't call for drawing the legacy nearest-point on any T007 image
# (the comparison lives in result.json's target_pixel_px instead), so this
# stays unused -- not deleted, in case a future task wants the visual.
_SELF_BODY_COLOR = (255, 140, 0)
_REGION_PALETTE = [
    (255, 0, 255), (0, 200, 255), (255, 140, 0), (255, 64, 64),
    (170, 120, 255), (255, 210, 0), (0, 160, 200), (200, 80, 160),
]


def _annotation_font(size=15):
    from PIL import ImageFont

    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _draw_label_box(draw, anchor_xy, text, accent_rgb, image_size, font=None, pad=4, offset=(14, -10)):
    """White-fill/black-text label box with an accent border; auto-flips off
    the image edge (docs/ANNOTATION_SPEC.md). Returns the box (x0,y0,x1,y1)."""
    font = font or _annotation_font()
    width, height = image_size
    ax, ay = anchor_xy
    bbox = draw.textbbox((0, 0), text, font=font)
    box_w = (bbox[2] - bbox[0]) + 2 * pad
    box_h = (bbox[3] - bbox[1]) + 2 * pad
    x, y = ax + offset[0], ay + offset[1]
    if x + box_w > width:
        x = ax - box_w - 14
    if y < 0:
        y = ay + 10
    x = max(0, min(x, width - box_w))
    y = max(0, min(y, height - box_h))
    draw.rectangle([x, y, x + box_w, y + box_h], fill=(255, 255, 255), outline=accent_rgb, width=2)
    draw.text((x + pad, y + pad), text, font=font, fill=(0, 0, 0))
    return x, y, x + box_w, y + box_h


def _draw_crosshair(draw, xy, accent_rgb, arm=9, gap=3, radius=7):
    """Double-stroked crosshair + ring: white outer stroke, accent inner."""
    x, y = xy
    for width, color, r in ((4, (255, 255, 255), radius + 1), (2, accent_rgb, radius)):
        draw.ellipse([x - r, y - r, x + r, y + r], outline=color, width=width)
        for x0, y0, x1, y1 in (
            (-(gap + arm), 0, -gap, 0), (gap, 0, gap + arm, 0),
            (0, -(gap + arm), 0, -gap), (0, gap, 0, gap + arm),
        ):
            draw.line([x + x0, y + y0, x + x1, y + y1], fill=color, width=width)


def _blend_mask(image, mask, color_rgb, alpha_255):
    """Alpha-blend color_rgb into image (PIL RGB) where mask is True."""
    from PIL import Image

    array = np.asarray(image.convert("RGB")).astype(np.float32)
    alpha = alpha_255 / 255.0
    array[mask] = array[mask] * (1 - alpha) + np.array(color_rgb, dtype=np.float32) * alpha
    return Image.fromarray(array.astype(np.uint8), "RGB")


def _mask_centroid(mask):
    if not np.any(mask):
        return None
    ys, xs = np.nonzero(mask)
    return int(xs.mean()), int(ys.mean())


def _object_color(index):
    return _REGION_PALETTE[index % len(_REGION_PALETTE)]


def _annotate_rgb_left(rgb_left, tabletop, run_dir):
    """T007: one crosshair + label per tabletop object, no single TARGET
    marker (docs/ANNOTATION_SPEC.md 'region_overlay -> T007 階段';
    tasks/T007.../TASK.md Step 6). Label box only -- no info bar."""
    from PIL import Image, ImageDraw

    image = Image.fromarray(rgb_left.astype(np.uint8)).convert("RGB")
    draw = ImageDraw.Draw(image)
    for index, obj in enumerate(tabletop["objects"]):
        color = _object_color(index)
        u, v = obj["representative_pixel_px"]
        _draw_crosshair(draw, (u, v), color)
        _draw_label_box(draw, (u, v), f"OBJ{obj['id']} ({u},{v})", color, image.size)
    image.save(os.path.join(run_dir, "rgb_left_annotated.png"))


def _annotate_rgb_color(rgb_color, measurement, tabletop, color_frame, color_intrinsics, run_dir):
    """Reprojects each object's surface_center_world_xyz_m into the color
    camera; objects that fall outside the frame are simply not drawn.
    target_outside_color_fov (the legacy single-target field, unchanged
    semantics -- tasks/T007.../TASK.md Step 5) is still computed from
    `measurement`, independent of what gets drawn here."""
    from PIL import Image, ImageDraw

    image = Image.fromarray(rgb_color.astype(np.uint8)).convert("RGB")
    draw = ImageDraw.Draw(image)
    for index, obj in enumerate(tabletop["objects"]):
        color = _object_color(index)
        projected = _camera_xyz_to_pixel(
            _world_xyz_to_camera_xyz(obj["surface_center_world_xyz_m"], color_frame), color_intrinsics
        )
        if projected is None or not (0 <= projected[0] < WIDTH and 0 <= projected[1] < HEIGHT):
            continue
        u, v = int(round(projected[0])), int(round(projected[1]))
        _draw_crosshair(draw, (u, v), color)
        _draw_label_box(draw, (u, v), f"OBJ{obj['id']} ({u},{v})", color, image.size)
    image.save(os.path.join(run_dir, "rgb_color_annotated.png"))
    legacy_world = measurement["target_world_xyz_m"]
    if legacy_world is None:
        return False
    legacy_projected = _camera_xyz_to_pixel(_world_xyz_to_camera_xyz(legacy_world, color_frame), color_intrinsics)
    return legacy_projected is None or not (0 <= legacy_projected[0] < WIDTH and 0 <= legacy_projected[1] < HEIGHT)


def _annotate_depth_left(axial_left, tabletop, self_body_max_m, run_dir):
    """Grayscale depth preview + self-body overlay + one crosshair per
    tabletop object, labelled with its straight-line distance from the
    camera (docs/ANNOTATION_SPEC.md, tasks/T007.../TASK.md Step 6)."""
    from PIL import Image, ImageDraw

    image = Image.fromarray(_depth_preview(axial_left), "RGB")
    self_body_mask = np.isfinite(axial_left) & (axial_left > 0) & (axial_left < self_body_max_m)
    if np.any(self_body_mask):
        image = _blend_mask(image, self_body_mask, _SELF_BODY_COLOR, 90)
    draw = ImageDraw.Draw(image)
    centroid = _mask_centroid(self_body_mask)
    if centroid is not None:
        _draw_label_box(draw, centroid, "SELF-BODY", _SELF_BODY_COLOR, image.size)
    for index, obj in enumerate(tabletop["objects"]):
        color = _object_color(index)
        u, v = obj["representative_pixel_px"]
        dist_m = obj["camera_to_surface_center_distance_m"]
        _draw_crosshair(draw, (u, v), color)
        _draw_label_box(draw, (u, v), f"OBJ{obj['id']} {dist_m:.3f} m", color, image.size)
    image.save(os.path.join(run_dir, "depth_left_annotated.png"))


_TABLE_COLOR = (160, 160, 160)


def _annotate_region_overlay(rgb_left, axial_left, self_body_max_m, tabletop, run_dir):
    """T007: table plane in grey, each object in its own REGION_PALETTE
    color, self-body still orange (docs/ANNOTATION_SPEC.md 'region_overlay
    -> T007 階段')."""
    from PIL import Image, ImageDraw

    valid = np.isfinite(axial_left) & (axial_left > 0)
    self_body_mask = valid & (axial_left < self_body_max_m)
    image = Image.fromarray(rgb_left.astype(np.uint8)).convert("RGB")
    image = _blend_mask(image, tabletop["table_mask"], _TABLE_COLOR, 60)
    for index, obj in enumerate(tabletop["objects"]):
        image = _blend_mask(image, obj["mask"], _object_color(index), 100)
    image = _blend_mask(image, self_body_mask, _SELF_BODY_COLOR, 90)
    draw = ImageDraw.Draw(image)
    table_centroid = _mask_centroid(tabletop["table_mask"])
    if table_centroid is not None:
        _draw_label_box(draw, table_centroid, "TABLE", _TABLE_COLOR, image.size)
    self_body_centroid = _mask_centroid(self_body_mask)
    if self_body_centroid is not None:
        _draw_label_box(draw, self_body_centroid, "SELF-BODY", _SELF_BODY_COLOR, image.size)
    for index, obj in enumerate(tabletop["objects"]):
        color = _object_color(index)
        u, v = obj["representative_pixel_px"]
        _draw_crosshair(draw, (u, v), color)
        _draw_label_box(draw, (u, v), f"OBJ{obj['id']} ({u},{v})", color, image.size)
    image.save(os.path.join(run_dir, "region_overlay.png"))


def _read_camera_intrinsics(stage, camera_path):
    """USD focalLength/horizontalAperture -> pinhole intrinsics. Touches USD
    (stage), not async. Shared by _build_measurement (left) and the color
    annotation's reprojection (T006 Step 7)."""
    from pxr import UsdGeom

    camera = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
    focal_length = camera.GetFocalLengthAttr().Get()
    horizontal_aperture = camera.GetHorizontalApertureAttr().Get()
    horizontal_fov_rad = 2 * math.atan(horizontal_aperture / (2 * focal_length))
    intrinsics = _intrinsics_from_fov(WIDTH, HEIGHT, horizontal_fov_rad)
    # _intrinsics_from_fov() is a generic pure function and labels itself
    # "geometric" regardless of where the FOV came from -- this call site
    # knows the actual provenance (T006 Step 3c), so it relabels source and
    # keeps the three raw numbers everything else here is derived from.
    intrinsics["source"] = "usd_camera_focal_length_and_aperture"
    intrinsics["raw"] = {
        "focal_length": focal_length,
        "horizontal_aperture": horizontal_aperture,
        "horizontal_fov_rad": horizontal_fov_rad,
    }
    return intrinsics


def _build_measurement(stage, axial_left, left_frame):
    """Calls the pure T006 functions, builds the measurement dict. Touches
    USD (stage) via _read_camera_intrinsics, not async."""
    intrinsics = _read_camera_intrinsics(stage, CAMERA_PATHS["left"])
    selection = _select_target_surface_pixel(axial_left)
    if selection["target_pixel_px"] is None:
        return {
            "target_pixel_px": None,
            "camera_to_target_distance_m": None,
            "camera_to_target_distance_cm": None,
            "target_camera_xyz_m": None,
            "target_world_xyz_m": None,
            "intrinsics": intrinsics,
            "self_body_exclusion": selection,
        }
    u, v = selection["target_pixel_px"]
    depth_m = float(axial_left[v, u])
    camera_xyz = _pixel_depth_to_camera_xyz(u, v, depth_m, intrinsics)
    distance_m = float(np.linalg.norm(camera_xyz))
    return {
        "target_pixel_px": [u, v],
        "camera_to_target_distance_m": distance_m,
        "camera_to_target_distance_cm": distance_m * 100.0,
        "target_camera_xyz_m": camera_xyz,
        "target_world_xyz_m": _camera_to_world_xyz(camera_xyz, left_frame),
        "intrinsics": intrinsics,
        "self_body_exclusion": selection,
    }


async def demo_capture_async(label=None) -> dict:
    """Capture one run: rgb_left/right/color.png, depth_axial_left.npy,
    depth_radial_left.npy, depth_preview.png, result.json.

    No measurement is performed here -- see docs/CAPTURE_REBUILD_PLAN.md.
    """
    import omni.kit.app
    import omni.timeline
    from pxr import Usd, UsdGeom

    timeline = omni.timeline.get_timeline_interface()
    timeline_playing_before = timeline.is_playing()

    setup_result = await demo_capture_setup()
    _, stage, stage_path, output_root = _active_stage_context()
    stage_sha256_before = _sha256(stage_path)
    run_id, run_dir, _timestamp, _label, _sequence = _create_unique_run_dir(
        output_root, label
    )
    annotators = _registry()["annotators"]
    app = omni.kit.app.get_app()

    # Hide before _wait_for_ik_settle() so Hydra has the whole settle window
    # to propagate the visibility change (rule 1, docs/CAPTURE_REBUILD_PLAN.md).
    target = stage.GetPrimAtPath(IK_TARGET_PATH)
    target_imageable = None
    previous_visibility = None
    ik_target_hidden = False
    if target.IsValid() and target.IsA(UsdGeom.Imageable):
        target_imageable = UsdGeom.Imageable(target)
        previous_visibility = target_imageable.GetVisibilityAttr().Get()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            target_imageable.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        ik_target_hidden = True

    try:
        settle = await _wait_for_ik_settle()
        for _ in range(5):
            await app.next_update_async()
        render_step_result = await _force_render_step()
        rgb = {}
        rgb_diagnostics = {}
        for eye in ("left", "right", "color"):
            rgb[eye], rgb_diagnostics[eye] = await _read_rgb_with_retry(
                annotators[eye]["rgb"]
            )
        axial_left = np.asarray(
            annotators["left"]["distance_to_image_plane"].get_data(),
            dtype=np.float32,
        )
        radial_left = np.asarray(
            annotators["left"]["distance_to_camera"].get_data(),
            dtype=np.float32,
        )
    finally:
        # Only restore visibility here -- no cleanup call (rule 2,
        # docs/CAPTURE_REBUILD_PLAN.md: the reference project never tears
        # down its render graph, and this project's earlier cleanup-after-
        # every-capture design was the likely cause of "can't drag the
        # target after a capture", see docs/CAPTURE_RETROSPECTIVE.md
        # Finding 3).
        if target_imageable is not None:
            restored = previous_visibility or UsdGeom.Tokens.inherited
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                target_imageable.GetVisibilityAttr().Set(restored)

    _save_capture_products(run_dir, rgb, axial_left, radial_left)

    frames = {name: _camera_frame(stage, path) for name, path in CAMERA_PATHS.items()}
    arm_base_world_xyz_m = _arm_base_world_xyz(stage)
    baseline = float(
        np.linalg.norm(
            np.asarray(frames["left"]["position_m"])
            - np.asarray(frames["right"]["position_m"])
        )
    )
    green_audit = {eye: _green_audit_eye(rgb[eye]) for eye in ("left", "right", "color")}
    # green_audit is computed on the as-saved rgb_left (never on an annotated
    # copy -- Step 7 annotation happens after this, on a separate in-memory
    # copy of the array). This is this project's one quality gate; see
    # _collect_failures() and reviews/T004.md judgment B. Do not loosen.
    measurement = _build_measurement(stage, axial_left, frames["left"])
    tabletop = _build_tabletop_objects(axial_left, rgb["left"], measurement["intrinsics"], frames["left"], arm_base_world_xyz_m)
    failures = _collect_failures(rgb_diagnostics, settle, green_audit)
    failures.extend(tabletop["failures"])
    if measurement["target_pixel_px"] is None:
        failures.append("no target beyond self-body envelope")
    stage_sha256_after = _sha256(stage_path)

    # T006 Step 7b / T008 Step 0: catches the file on disk having changed
    # since demo_start.py's bootstrap exec (the M6 Finding 3 pattern) --
    # unknown (None) is reported as unknown, never coerced to false.
    loaded_sha256 = _loaded_source_sha256_at_bootstrap()
    disk_sha256 = _source_sha256_or_none()
    stale = None if loaded_sha256 is None or disk_sha256 is None else loaded_sha256 != disk_sha256
    source_integrity = {
        "loaded_sha256": loaded_sha256,
        "disk_sha256": disk_sha256,
        "stale": stale,
    }
    if stale is True:
        failures.append("loaded capture module is stale -- re-run demo_start.py")

    color_intrinsics = _read_camera_intrinsics(stage, CAMERA_PATHS["color"])
    _annotate_rgb_left(rgb["left"], tabletop, run_dir)
    outside_color_fov = _annotate_rgb_color(rgb["color"], measurement, tabletop, frames["color"], color_intrinsics, run_dir)
    _annotate_depth_left(axial_left, tabletop, SELF_BODY_MAX_CAMERA_DISTANCE_M, run_dir)
    _annotate_region_overlay(rgb["left"], axial_left, SELF_BODY_MAX_CAMERA_DISTANCE_M, tabletop, run_dir)
    images = {
        "rgb_left_annotated": "rgb_left_annotated.png",
        "rgb_color_annotated": "rgb_color_annotated.png",
        "depth_left_annotated": "depth_left_annotated.png",
        "region_overlay": "region_overlay.png",
    }
    result = _build_result_json(
        run_id,
        failures,
        frames["left"]["position_m"],
        arm_base_world_xyz_m,
        measurement,
        images,
        source_integrity,
        outside_color_fov,
        tabletop,
    )
    diagnostics = _build_diagnostics_json(
        green_audit,
        {"setup": setup_result, "step": render_step_result},
        frames,
        stage_sha256_before,
        stage_sha256_after,
        timeline_playing_before,
        ik_target_hidden,
        baseline,
        settle,
    )
    with open(os.path.join(run_dir, "result.json"), "w", encoding="utf-8") as output:
        json.dump(result, output, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, "diagnostics.json"), "w", encoding="utf-8") as output:
        json.dump(diagnostics, output, indent=2, ensure_ascii=False)
    print(result)
    return result


async def _scheduled_capture_wrapper(label) -> dict:
    registry = _registry()
    try:
        result = await demo_capture_async(label)
    except Exception as exc:
        result = {
            "status": "error",
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
    registry["last_capture_result"] = result
    registry["capture_task"] = None
    return result


def demo_capture(label=None) -> dict:
    """GUI-safe entry point: schedules one capture, returns immediately.

    Use demo_capture_status() to read the result once it's done.
    """
    registry = _registry()
    task = registry.get("capture_task")
    if task is not None and not task.done():
        return {"status": "already_scheduled"}
    task = asyncio.ensure_future(_scheduled_capture_wrapper(label))
    registry["capture_task"] = task
    return {"status": "scheduled"}


def demo_capture_status() -> dict:
    """Result of the most recent demo_capture() call."""
    registry = _registry()
    task = registry.get("capture_task")
    if task is not None and not task.done():
        return {"status": "capturing"}
    last = registry.get("last_capture_result")
    if last is None:
        return {"status": "not_captured"}
    return last
