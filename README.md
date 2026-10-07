# cone_calib

Base-to-base calibration of a bimanual setup (currently a Franka **FR3** and a
Universal Robots **UR3e**) using a fixed aluminium plate with K ≥ 3 conical
seats. A human hand-guides each robot so a sphere/hemisphere-tipped probe sits
in a seat and triggers a capture at several tool orientations per seat; the
offline solver recovers each robot's seat centres (and optional probe-tip
offset) by **pivot calibration**, then the rigid transform between the two
bases by **Kabsch/Umeyama registration**, plus CAD sanity checks.

> **This package never commands robot motion. It only reads `/joint_states` and
> TF.** Add a robot by adding one `config/<robot_id>.yaml` — no code changes.

See `docs/2026-10-07_CONE_CALIB_DESIGN.md` for the full design and
`docs/exploration.md` for the workspace facts it is built on.

## Conventions

- **Quaternions are `(x, y, z, w)` (scipy / XYZW) everywhere.** All rotation math
  goes through `scipy.spatial.transform.Rotation` via `cone_calib/se3.py`.
- `T_A_B` is the pose of frame `B` in frame `A`: `x_A = T_A_B @ x_B`.
- Reference robot **A = UR3e** (`ur3e_base_link`), so the reported `T_A_B` is the
  FR3 base (`fr3_link0`) expressed in the UR3e base — directly comparable to the
  existing `world_to_base` in `arms_data_collection/config/static_tfs.yaml`.

## Layout

| Module | ROS? | Purpose |
|---|---|---|
| `se3.py` | no | SE(3)/quaternion helpers, Kabsch/Umeyama |
| `pivot.py` | no | pivot calibration + diagnostics/observability |
| `registration.py` | no | inter-robot registration + CAD sanity |
| `io.py` | no | session/result I/O, static_tfs + xacro emitters |
| `kinematics.py` | ros | TF pose source + joint mapping (FK stubbed) |
| `capture_node.py` | ros | settle/average/save capture node |
| `solve.py` | no | offline CLI: sessions → results |

## Build

```bash
cd /home/swisscat/arms_ws
colcon build --packages-select cone_calib --symlink-install
source install/setup.bash
```

## Capture (per robot)

Bring the robot up first (FR3 via the py_panda2 bimanual launch; **UR3e via the
repo launches** so the factory calibration is loaded — confirm no kinematics
hash warning at bringup). Then, for the interactive terminal UX, run the node
directly so stdin is attached:

```bash
ros2 run cone_calib capture_node --ros-args \
  -p config_file:=$(ros2 pkg prefix cone_calib)/share/cone_calib/config/ur3e.yaml \
  -p plate_config:=$(ros2 pkg prefix cone_calib)/share/cone_calib/config/plate.yaml
```

Commands: `<Enter>` = capture the current seat (waits for settle, averages a
window), `n` = next seat, `u` = undo the last capture, `s` = save, `q` =
quit+save, `seat <id>` = jump to a seat. Raw samples are written incrementally
to `data/cone_calib/<robot_id>_<timestamp>/session.yaml`.

Service-driven mode (e.g. from a script or `ros2 service call`):

```bash
ros2 launch cone_calib capture.launch.py robot_id:=ur3e
ros2 service call /cone_calib_capture/capture std_srvs/srv/Trigger
ros2 service call /cone_calib_capture/next_seat std_srvs/srv/Trigger
```

Repeat for every seat (8–15 orientations each), then for the other robot.

## Solve (offline)

```bash
ros2 run cone_calib solve \
  --session data/cone_calib/ur3e_.../session.yaml \
  --session data/cone_calib/fr3_.../session.yaml \
  --plate $(ros2 pkg prefix cone_calib)/share/cone_calib/config/plate.yaml \
  --cross-check-static-tfs src/arms_data_collection/config/static_tfs.yaml \
  --out cone_calib_results.yaml
```

The results YAML contains `T_A_B` (translation, XYZW quaternion, RPY deg, 4×4
matrix), estimated tip offsets, seat centres, all diagnostics/warnings, a
`static_tfs.yaml`-schema entry to paste into the single source of truth, a xacro
`<joint>` snippet, and a ready-to-run publish command. The solver also prints a
cross-check of `T_A_B` against `world_to_base` (targets: linear ≤ 2 mm, angular
≤ 0.05 rad).

## Publish the result

```bash
ros2 launch cone_calib publish_calibration.launch.py \
  parent:=ur3e_base_link child:=fr3_link0 \
  x:=... y:=... z:=... qx:=... qy:=... qz:=... qw:=...
```

(The exact command with the solved numbers is printed by `solve` and stored in
the results file.)

## Tests

ROS-free math tests run with plain pytest:

```bash
cd src/cone_calib && python3 -m pytest test/test_se3.py test/test_pivot.py \
  test/test_registration.py test/test_conventions.py test/test_io.py -q
```

## Status / not yet implemented

- `pose_source: fk` and `--recompute-fk` are guarded stubs (the workspace has no
  active FK; a correct UR recompute must be fed `my_robot_calibration.yaml`).
- Bench validation on the real robots (design doc PR-5) is pending.
