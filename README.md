# cone_calib

Base-to-base calibration of a bimanual setup (a Franka **FR3** and a Universal
Robots **UR3e**) using a fixed aluminium plate with K ≥ 3 conical seats. A human
hand-guides each robot so a sphere-tipped probe sits in a seat and triggers a
capture at several tool orientations per seat; the offline solver then recovers
each robot's seat centres (and the probe-tip offset) by **pivot calibration**,
the rigid transform between the two bases by **Kabsch/Umeyama registration**,
and runs CAD sanity checks.

> **This package only reads robot state (`/joint_states` + TF). It never
> commands motion.** Add a robot by adding one `config/<robot_id>.yaml` — no
> code changes.

Design rationale and the workspace facts it is built on:
`docs/2026-10-07_CONE_CALIB_DESIGN.md` and `docs/exploration.md`.

---

## Conventions

- **Quaternions are `(x, y, z, w)` (scipy / XYZW)** everywhere a quaternion
  crosses a boundary.
- `T_A_B` is the pose of frame `B` expressed in frame `A`: `x_A = T_A_B @ x_B`.
- Reference robot **A = UR3e** (`ur3e_base_link`); the reported `T_A_B` is the
  FR3 base (`fr3_link0`) expressed in the UR3e base — directly comparable to the
  existing `world_to_base` in `arms_data_collection/config/static_tfs.yaml`.
- `world` is aliased to `ur3e_base_link` (the UR driver publishes `world →
  ur3e_base_link`).

---

## 0. Prerequisites (before any run)

1. **Source the workspace:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/arms_ws/install/setup.bash
   ```
2. **FR3 kinematic calibration — do this FIRST.** The FR3's nominal DH has
   per-unit error, so the `fr3_link0 → fr3_link8` pose from
   `robot_state_publisher` is off until the arm's kinematics are calibrated.
   Run the FR3 kinematic calibration and load its result *before* capturing:
   <https://github.com/platonics-delft/kinematics_calibration>. (This is the FR3
   analogue of the UR3e's factory per-unit calibration in step 3 — without it
   the FR3 flange poses, and therefore the whole result, carry a systematic
   kinematic error.)
3. **Bring both robots up** with their normal launches: FR3 via the py_panda2
   bimanual launch (with the calibrated kinematics from step 2); **UR3e via the
   repo launch** — `ur3e_control` — so `robot_state_publisher` carries the
   per-unit factory kinematics calibration (the stock UR launch uses nominal DH
   and is several mm off). Confirm the UR driver logs **no kinematics-hash
   mismatch** at bringup.
4. **Confirm the UR joint order once** (it is not pinned in config):
   ```bash
   ros2 topic echo /joint_states --once
   ```
5. **Fill in the real plate geometry** in `config/plate.yaml`:
   `seats_nominal` (CAD coordinates in the plate frame, metres) and
   `orthogonal_triple`. The probe `tip_offset` is already set to the measured
   **62.5 mm** (TCP → sphere centre along flange +z) for both robots.

---

## 1. Build

```bash
cd ~/arms_ws
colcon build --packages-select cone_calib --symlink-install
source install/setup.bash
```
(`--symlink-install` means later edits to `.py`/YAML are picked up on the next
run without rebuilding; only adding a new executable needs a rebuild.)

---

## 2. Capture — run **once per robot**

Hand-guide the probe into a seat (FR3 hand-guiding / UR freedrive), let the arm
settle, and press **Enter** to capture. Repeat at 8–15 tool orientations per
seat; `n` moves to the next seat. Do every seat, then repeat for the other
robot.

```bash
# UR3e (swap ur3e -> fr3 for the Franka):
ros2 run cone_calib capture_node --ros-args \
  -p config_file:=$(ros2 pkg prefix cone_calib)/share/cone_calib/config/ur3e.yaml \
  -p plate_config:=$(ros2 pkg prefix cone_calib)/share/cone_calib/config/plate.yaml
```

Terminal keys: `<Enter>` = capture the current seat · `n` = next seat ·
`u` = undo last · `s` = save · `q` = quit + save · `seat <id>` = jump to a seat.

- Raw samples stream to `~/arms_ws/data/cone_calib/<robot>_<timestamp>/session.yaml`
  (incremental — a crash loses at most one capture).
- After each capture it prints the running count, the tilt spread, and (once ≥ 3
  captures on a seat) the running **per-divot RMS**.

**Settle tuning.** The node waits until all joint velocities stay below
`joint_vel_threshold` for `settle_time`, then averages a window. The FR3's
cartesian impedance controller holds with residual jitter (~0.004–0.006 rad/s),
so its default threshold is **0.012 rad/s** (UR3e 0.005). The active values are
logged at startup. If you get `settle timeout`, the message shows the current
`max|vel|`; raise the threshold live (no rebuild):
```bash
  ... -p joint_vel_threshold:=0.02
```
If captures fire while you are still nudging the arm, lower it. `settle_time`,
`average_window`, and `settle_timeout` are overridable the same way.

---

## 3. Solve (offline)

Point `--session` at each robot's `session.yaml`:

```bash
ros2 run cone_calib solve \
  --session ~/arms_ws/data/cone_calib/ur3e_*/session.yaml \
  --session ~/arms_ws/data/cone_calib/fr3_*/session.yaml \
  --plate   $(ros2 pkg prefix cone_calib)/share/cone_calib/config/plate.yaml \
  --cross-check-static-tfs ~/arms_ws/src/arms_data_collection/config/static_tfs.yaml \
  --out     ~/arms_ws/data/cone_calib/results.yaml
```

Prints and stores `T_A_B` (translation, XYZW quaternion, RPY deg, 4×4 matrix),
each robot's estimated tip offset + overall and **per-divot RMS**, the
**orthogonality deviation from 90°** per robot, the inter-seat/two-route sanity
checks, the cross-check vs `world_to_base` (targets: linear ≤ 2 mm, angular ≤
0.05 rad), a `static_tfs.yaml`-schema entry to paste in, a xacro `<joint>`
snippet, and a ready-to-run publish command. Optional `--reference-robot <id>`
overrides which base is A.

---

## 4. Publish the solved transform

Copy the exact command `solve` printed, or:

```bash
ros2 launch cone_calib publish_calibration.launch.py \
  parent:=ur3e_base_link child:=fr3_link0 \
  x:=... y:=... z:=... qx:=... qy:=... qz:=... qw:=...
```

---

## 5. Cross-robot sanity check — `echo_sphere` (one robot at a time)

A quick, pipeline-independent check: touch the **same physical point** with each
robot and compare where it lands in the shared `world` frame. The difference is
the base-to-base error, in metres. `echo_sphere` reads `world → flange` from TF,
applies the tip offset, and prints the sphere centre in `world`.

**Prerequisite:** the `world → flange` chain must be published — each robot's
`robot_state_publisher` plus the base-to-base transform under test (either the
`arms_data_collection` static_tfs publisher for the existing calibration, or
cone_calib's `publish_calibration.launch.py` for a fresh result).

Both robots can't occupy the same point at once, so do it **sequentially**:

**Step 1 — robot A.** Run it, drive A's probe to the point, read the printed
vector:
```bash
ros2 run cone_calib echo_sphere --ros-args \
  -p config_file:=$(ros2 pkg prefix cone_calib)/share/cone_calib/config/fr3.yaml
# -> [fr3] sphere@world = [ 0.312450,  0.104300,  0.058700] m
```
Note that vector, then `Ctrl-C`.

**Step 2 — robot B, with A's reading as the reference.** Drive B's probe to the
**same** point; the tool live-prints the distance to A so you can watch it
shrink as you align and read the final error:
```bash
ros2 run cone_calib echo_sphere --ros-args \
  -p config_file:=$(ros2 pkg prefix cone_calib)/share/cone_calib/config/ur3e.yaml \
  -p reference:="[0.312450, 0.104300, 0.058700]"
# -> [ur3e] sphere@world = [ 0.311900, ...] m   d(ref) =   0.74 mm   delta = [-0.6, -0.5, +0.4] mm
```
`d(ref)` (and the per-axis `delta`) is the base-to-base discrepancy at that
point. Repeat at a few points around the workspace.

Other overrides: `-p world_frame:=ur3e_base_link`, `-p flange_frame:=...`,
`-p tip_offset:="[0.0, 0.0, 0.0625]"`, `-p rate:=5.0`. If you see
`TF world -> <flange> unavailable`, the base-to-base leg of the chain isn't
being published yet.

---

## 6. Tests

ROS-free math + integration tests run with plain pytest:
```bash
cd ~/arms_ws/src/cone_calib
python3 -m pytest test/test_se3.py test/test_pivot.py test/test_registration.py \
  test/test_conventions.py test/test_io.py test/test_solve.py -q
```
`test_flake8.py` is green. `test_pep257.py` reports docstring-style notices
(D213/section format) — left in the readable sibling style per the workspace
convention (the other in-house packages run pep257 red too); not a code defect.

---

## 7. Troubleshooting

- **`settle timeout`** — the velocity threshold is below the controller's
  resting jitter. Raise it: `-p joint_vel_threshold:=0.02` (see §2).
- **`TF world -> <flange> unavailable`** — the base-to-base transform (or a
  robot's `robot_state_publisher`) isn't running. Start the static_tfs publisher
  or `publish_calibration.launch.py` (§5 prerequisite).
- **UR looks a few mm off everywhere** — it was likely brought up with the stock
  launch (nominal DH). Relaunch via the `ur3e_control` repo launch so the
  factory `my_robot_calibration.yaml` is loaded (§0).
- **Pivot refuses to estimate the tip offset** — not enough orientation
  diversity in a seat (tilt about varied axes). Capture more, varied tilts, or
  set `estimate_tip_offset: false` to use the measured 62.5 mm directly.

---

## 8. Status / not yet implemented

- `pose_source: fk` and `solve --recompute-fk` are guarded stubs (the workspace
  has no active FK; a correct UR recompute must be fed `my_robot_calibration.yaml`).
- Bench validation on the real robots (design doc PR-5): **done (2026-10-07)**.
