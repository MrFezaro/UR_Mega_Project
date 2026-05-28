# Cube Pointer System Documentation

A ROS 2 pipeline that uses a wrist-mounted USB camera to detect coloured cubes on a table and direct a UR5e arm to point at each one in sequence.

---

## Architecture Overview

```
/image_raw  ──►  ColorSquareDetector  ──►  /cube_detections (JSON)
                                                    │
                                           CubePointerNode
                                                    │
                                      /urscript_interface/script_command
                                                    │
                                               UR5e arm
```

Three ROS 2 nodes are launched together:

| Node | Package | Role |
|------|---------|------|
| `usb_cam` | `usb_cam` | Streams 640×480 @ 20 fps from `/dev/video0` |
| `color_square_detector` | `color_square_detector` | Detects coloured squares, publishes centroids |
| `cube_pointer` | `ur5e_controller` | State machine that moves the arm to each cube |

---

## 1. Computer Vision

### Pipeline per frame

```
BGR frame
   │
   ▼
GaussianBlur(7×7)
   │
   ▼
cvtColor to HSV
   │
   ├── Red mask   (dual-range OR, wraps hue wheel)
   ├── Green mask
   └── Blue mask
          │
          ▼
   morphologyEx  (OPEN to CLOSE, 5×5 kernel)
          │
          ▼
   findContours
          │
          ▼
   Filter candidates (4 checks below)
          │
          ▼
   Best centroid per colour to JSON to /cube_detections
```

### Why HSV?

HSV separates *hue* (colour identity) from *saturation* and *value* (lighting), making thresholds far less sensitive to illumination changes than BGR.

### Square validation

Each contour passes four checks before being accepted:

| Check | Parameter | Default | Purpose |
|-------|-----------|---------|---------|
| Minimum area | `min_area` | 1500 px² | Reject noise speckles |
| Polygon corners | `approx_eps` | 0.04 | Must approximate to exactly 4 vertices |
| Aspect ratio | `aspect_min` / `aspect_max` | 0.5 to 2.0 | Bounding box cannot be extremely elongated |
| Side uniformity | `side_ratio_min` | 0.60 | Shortest side divided by longest side must be at least 0.60 |

The largest valid contour per colour wins. Its centroid comes from `minAreaRect`.

### Red dual HSV range

Red wraps around the HSV hue wheel (0° = 360°), so two ranges are used and OR-ed:

```
Range 1:  H  0–10,  S 100–255,  V 100–255
Range 2:  H 160–180, S 100–255, V 100–255
```

### Detection message format

Published as JSON on `/cube_detections` every frame:

```json
{"width": 640, "height": 480, "Red": [312, 241], "Green": null, "Blue": [489, 130]}
```

`null` means the colour was not detected in that frame.

### HSV tuning mode

Set `tune_mode: true` and `tune_color: "Green"` to open a side-by-side live window (camera | mask) with six trackbars. Press **S** to save the result to `color_params.yaml`.

---

## 2. Motion Planning

### Pixel to robot coordinates

The camera is rigidly mounted on the TCP and points straight down. A ray is cast from the pixel through the pinhole camera model and intersected with the table plane:

```
1.  ray_cam  = [(u - cx)/fx,  (v - cy)/fy,  1.0]          # camera frame
2.  ray_base = R_tool @ R_cam @ ray_cam                    # rotate to robot base
3.  t        = (table_z - tcp_z) / ray_base[2]             # ray-plane intersection
4.  xyz      = tcp_xyz + t * ray_base                      # world position
```

`R_tool` comes from the TCP rotation vector. `R_cam` applies the `camera_yaw_deg` offset for any rotational misalignment between the camera and tool frame.

Camera intrinsics (`camera_fx`, `camera_fy`, `camera_cx`, `camera_cy`) are loaded from `ost.yaml`, produced by a checkerboard calibration run with `camera_calibration`.

### State machine

```
        ┌──────────────────────────────────────────┐
        │                                          │
        ▼                                          │ color_idx = 0
      HOME  ──movej to overview pose──►  DETECT
                                            │
                              ┌─── found ───┤
                              │             │
                              │         not found
                              │             │
                              │             ▼
                              │          SEARCH  ◄── +45° then -45°
                              │             │
                              │    ┌─ found ┤
                              │    │        │ still not found
                              │    │        ▼
                              │    │      RETREAT (skip)
                              │    │        │
                              ▼    ▼        │
                           APPROACH         │
                              │             │
                              └────────────►┘
                                         RETREAT
                                            │
                                   more colours?
                                     │       │
                                    yes      no
                                     │       ▼
                                     │     DONE to restart (HOME, color_idx=0)
                                     ▼
                                    HOME
```

### State descriptions

| State | What happens |
|-------|-------------|
| `HOME` | `movej` to the overview pose, detection buffer flushed |
| `DETECT` | Waits up to 6 s for a detection message, extracts the target colour's pixel and projects it to robot XYZ |
| `SEARCH` | Rotates TCP position ±45° around the robot base Z axis and tries again (up to `len(search_angles_deg)` attempts) |
| `APPROACH` | `movej` to `[x, y, table_z + cube_height + approach_height]`, hovering 10 cm above the cube |
| `RETREAT` | Advances `color_idx`, goes to `DONE` after all three colours |
| `DONE` | Logs completion and restarts from `HOME` |

### URScript motion call

```python
movej(p[x, y, z, rx, ry, rz], a=1.2, v=1.05, t=<duration>)
```

The `p[...]` prefix tells the UR controller to interpret the target as a Cartesian TCP pose and solve IK internally. Synchronisation is handled with `time.sleep(duration + 1.0)` as there is no explicit acknowledgement from the controller.

---

## 3. Error Handling and Known Limitations

### Explicitly handled

- **Cube not found after all search poses** logs `"{colour}: not found, skipping"` and advances to the next colour
- **Detection timeout** treated identically to no detection
- **Invalid ray-cast** (ray points away from the table, `t < 0`) returns `None` and is treated as no detection

### Known weaknesses
| Issue | Root cause | Effect |
|-------|-----------|--------|
| No motion feedback | `sleep`-based sync instead of controller acknowledgement | Next move may start before the previous one finishes |
| Single-frame detection | `_wait_detection` returns the first message received | A false detection yields a wrong target position |
| Fixed search rotation | Search angles rotate the *home position*, not the cube's direction | May not help if the cube is directly in front of the robot |
| Lighting sensitivity | Static HSV thresholds, no adaptive normalisation | Changed ambient light made detection harder |
| Coordinate frame mismatch | Incorrect axis mapping or hardcoded `camera_yaw_deg` | Systematic XY offset, projected positions inverted/flipped |

### Suggested improvements
- **Temporal filtering** average centroids over N consecutive frames before projecting to reduce single-frame noise
- **Controller feedback** subscribe to `/joint_states` or the UR dashboard socket to confirm motion completion instead of relying on `sleep`
- **Adaptive thresholding** normalise V-channel (CLAHE) before HSV thresholding to handle varying illumination
- **Frame calibration** perform an explicit hand-eye calibration to resolve the coordinate frame mismatch and eliminate the systematic position offset
- **Depth integration** a RealSense or similar sensor would make pixel-to-XYZ projection exact and remove the dependency on a fixed table height

---

## Parameters

### `color_square_detector`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_area` | int | 1500 | Minimum contour area in pixels |
| `approx_eps` | double | 0.04 | Polygon approximation epsilon (fraction of perimeter) |
| `side_ratio_min` | double | 0.60 | Min ratio of shortest to longest side |
| `aspect_min` / `aspect_max` | double | 0.50 / 2.00 | Bounding-box aspect ratio limits |
| `tune_mode` | bool | false | Enable live HSV tuning window |
| `tune_color` | string | `"Red"` | Colour to tune |
| `params_file` | string | `color_params.yaml` | HSV parameter save/load path |

### `cube_pointer`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `home_pose` | double[6] | `[-0.283, -0.472, 0.644, -1.207, -2.896, -0.010]` | TCP pose at overview position `[x, y, z, rx, ry, rz]` |
| `motion_duration` | double | 6.0 s | Time budget for each `movej` call |
| `approach_height` | double | 0.10 m | Hover height above cube top |
| `table_z` | double | -0.016 m | Table surface Z in robot base frame |
| `cube_height` | double | 0.05 m | Cube height (used with `approach_height`) |
| `camera_yaw_deg` | double | 180.0° | Camera rotation offset relative to tool frame |
| `search_angles_deg` | double[] | `[45.0, -45.0]` | Rotation angles for search sweep |
| `camera_fx` / `camera_fy` | double | from `ost.yaml` | Focal lengths in pixels |
| `camera_cx` / `camera_cy` | double | from `ost.yaml` | Principal point in pixels |

---

## Launch

```bash
ros2 launch ur5e_controller cube_pointer.launch.py
```

To send the arm to its home joint configuration before starting:

```bash
ros2 run ur5e_controller home_position_node
```

To tune HSV parameters for a colour:

```bash
ros2 run color_square_detector detector_node \
  --ros-args -p tune_mode:=true -p tune_color:=Green
# Press S in the tuning window to save
```
