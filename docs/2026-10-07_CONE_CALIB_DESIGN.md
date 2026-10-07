# cone_calib — base-to-base calibration via conical seats (design)

**Date:** 2026-10-07
**Package:** `cone_calib` (new, `ament_python`, `src/cone_calib/`)
**Branch:** `feature/cone-calib` (from `dev`)
**Companion:** [`exploration.md`](exploration.md) — the Step-0 workspace reconnaissance this design is built on. Read it for evidence/citations; this doc assumes its findings.
**Status:** DESIGN — awaiting operator sign-off. **No code until this doc is approved** (project design-doc-first rule).

---

## 1. Goal & scope

Calibrate the rigid transform between the bases of a bimanual setup (currently a Franka **FR3** and a Universal Robots **UR3e**; later a UR5e + a UR3e) using a fixed aluminium plate with K ≥ 3 conical seats. A human hand-guides each robot so a sphere/hemisphere-tipped probe sits in a seat, triggers a capture at 8–15 tool orientations per seat, and repeats for every seat with each robot. Offline, we solve a **pivot calibration** per robot (seat centres + optional probe-tip offset), then a **rigid registration** between robots, plus CAD **sanity checks**.

**Hard constraints**
- **The package NEVER commands robot motion. It only reads state** (`/joint_states` + TF). No publishers to controllers, no action clients, no service calls that move anything.
- **Robot-agnostic:** adding a robot = adding one YAML file, **no code changes**. All robot specifics (frames, topics, joint names, tip offset) live in `config/<robot_id>.yaml`.
- All math lives in **ROS-free** modules (`se3.py`, `pivot.py`, `registration.py`) so it is unit-testable and runnable offline.
- **Quaternion convention is XYZW everywhere** (scipy `Rotation`), matching every first-party module in the workspace. Stated in the README and enforced in `se3.py`.

**Non-goals**
- No custom ROS messages in the first cut (see §7.1 trigger-service decision).
- No live auto-publishing of the result into the running TF tree beyond generating a ready-to-run `static_transform_publisher` launch + a xacro `<joint>` snippet.
- Editing the UR driver launch / `my_robot_calibration.yaml` provenance is **out of scope** (a `ur3e_control` concern, needs its own design doc). We *consume* whatever calibration the running driver loaded.

---

## 2. Resolved decisions (from exploration §9)

| # | Decision |
|---|---|
| D1 | **UR3e flange = `ur3e_tool0`** (URDF FK, factory-calibrated, deterministic; independent of the pendant TCP). |
| D2 | **Reference robot A = UR3e (`ur3e_base_link`)** ⇒ reported `T_A_B` = FR3 base (`fr3_link0`) expressed in the UR3e base. Directly comparable to the existing `world_to_base`. A `--reference-robot` flag keeps both directions derivable. |
| D3 | Package name **`cone_calib`**. |
| D4 | **Declare `numpy`/`scipy`** in `package.xml` (stricter than siblings, more correct). |
| D5 | **`pose_source: tf`** is the primary path for both robots (TF via `robot_state_publisher`); FR3's `O_T_EE` topic is the Desk EE (spatula), **not** the flange, so it is not used. `--recompute-fk` is deferred (§6.4). |

---

## 3. Notation & conventions

| Symbol | Meaning |
|---|---|
| `B` | robot base frame (param `base_frame`) |
| `F` | last link / flange frame (param `flange_frame`) |
| `R_i, p_i` | rotation & position of `F` in `B` at capture `i` (from TF or FK) |
| `t` | tip-sphere centre expressed in `F` (3-vector, param `tip_offset`) |
| `c_k` | centre of seat `k` expressed in `B` (3-vector) |
| `T_A_B` | 4×4 pose of frame `B` expressed in frame `A`, so `x_A = T_A_B @ x_B` |

Quaternions are stored and exchanged as **`(x, y, z, w)`** (scipy convention). `se3.py` is the single place that touches quaternion order; every other module goes through it.

---

## 4. Math

### 4.1 Pivot calibration (per robot, all seats solved jointly)

For capture `i` in seat `k`:

```
R_i t + p_i = c_k   ⟹   [ R_i | -E_k ] [ t ; c_1 ; … ; c_K ] = -p_i
```

`E_k` is a 3×3K selector with an identity block in column-block `k`. Stack all captures into one linear system with `3 + 3K` unknowns, solve by least squares (`numpy.linalg.lstsq`).

- **`estimate_tip_offset: true`** — solve for `t` and all `c_k` jointly (`t` shared across all seats).
- **`estimate_tip_offset: false`** — use `t` from the parameter; set `c_k = mean_i(R_i t + p_i)`.

**Diagnostics reported:**
- per-capture residual `||R_i t + p_i − c_k||`;
- RMS and max residual per seat and overall;
- singular values and condition number of the stacked matrix;
- estimated `t` vs the parameter value.

**Observability check.** Compute pairwise relative-rotation angles and axes within each seat. **Warn** if: max tilt spread < `min_tilt_spread_deg` (default 15°), **or** the relative-rotation axes are nearly parallel (e.g. all about the tool axis) — either makes `t` unobservable. If the condition number exceeds `max_condition_number`, **refuse** to solve for `t` with a clear message (fall back to fixed-`t` or abort per CLI flag).

**Outlier flagging (optional).** Flag captures with residual > `outlier_mad_k · MAD`. Flag in the report always; **drop only if `reject_outliers: true`**.

### 4.2 Registration between robots

Match seat centres between robot A and B **by seat ID** (string, e.g. `p1_s1`, `p2_s3`). Solve the rigid transform with **Kabsch/Umeyama** (no scale; handle the reflection case via `det` sign correction). With **D2**, A = UR3e, B = FR3, so the result is `T_A_B` = pose of FR3 base in UR3e base, with `x_A = T_A_B @ x_B`.

- Require ≥ 3 **non-collinear** shared seats. Check the smallest singular value of the centred point matrix; **raise** if degenerate (collinear/coincident).
- Report per-point residuals after alignment and the RMS.
- Support > 3 seats and multiple plate placements; correspondence is purely by ID.

### 4.3 Sanity checks

- Compare **inter-seat distances** measured by each robot vs nominal CAD distances (`seats_nominal`, in a plate frame) and vs each other. Report max deviation.
- Optionally register each robot's seat centres to the CAD nominal → `T_B_plate` per robot, giving an independent route `T_A_B = T_A_plate @ inv(T_B_plate)`. Report the discrepancy between the two routes.
- **Cross-check vs the existing pipeline:** compare `T_A_B` against `world_to_base` from `src/arms_data_collection/config/static_tfs.yaml` (parent `world`≡`ur3e_base_link`, child `base`≡ FR3 root ≈ `fr3_link0`), modulo the `base`↔`fr3_link0` and `world`↔`ur3e_base_link` aliases. Report linear/angular diff using the house metric (linear = `||Δt||`, angular = `2·acos(|Δq_w|)`; reference thresholds **linear ≤ 2 mm, angular ≤ 0.05 rad**). This is an independent validation, **not** an equality to assume.

---

## 5. Package layout

```
src/cone_calib/
  cone_calib/
    __init__.py
    se3.py            # SE(3) helpers, quaternion (XYZW) conversions, Kabsch/Umeyama   [ROS-free]
    pivot.py          # pivot calibration + diagnostics                                 [ROS-free, numpy]
    registration.py   # inter-robot registration + sanity checks                        [ROS-free, numpy]
    io.py             # load/save raw session data + results (YAML/JSON)                [ROS-free]
    kinematics.py     # flange pose source abstraction (TF lookup | FK from joints)     [ROS]
    capture_node.py   # ROS node: settle detection, averaging, saving raw samples        [ROS]
    solve.py          # offline CLI: raw data -> results                                 [ROS-free entry]
  config/
    fr3.yaml
    ur3e.yaml
    plate.yaml
  launch/
    capture.launch.py
    publish_calibration.launch.py
  test/
    test_se3.py  test_pivot.py  test_registration.py  test_conventions.py
    test_flake8.py  test_pep257.py  test_copyright.py   # ament stubs
  resource/cone_calib         # 0-byte marker
  docs/
    exploration.md
    2026-10-07_CONE_CALIB_DESIGN.md   # this file
  setup.py  setup.cfg  package.xml
  README.md
```

Scaffold by copying `src/arms_data_collection`'s `setup.py`/`setup.cfg`/`package.xml` and renaming. `data_files` keeps the `glob('launch/*launch.[pxy][yma]*')` + `glob('config/*.yaml')` pattern. `console_scripts`: `capture_node`, `solve`. **Build:** `cd /home/swisscat/arms_ws && colcon build --packages-select cone_calib --symlink-install && source install/setup.bash`.

---

## 6. Parameters (corrected to the real workspace — exploration §6)

### `config/fr3.yaml`
```yaml
robot_id: fr3
base_frame: fr3_link0
flange_frame: fr3_link8
joint_states_topic: /fr3/franka_state_publisher/joint_states   # NOT /franka/... ; namespace is fr3
joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7]
pose_source: tf                 # tf | fk
tip_offset: [0.0, 0.0, 0.107]   # tip-sphere centre in flange frame [m], PLACEHOLDER — edit to the real probe
estimate_tip_offset: true
tip_radius: 0.004               # [m], for reporting only
settle:
  joint_vel_threshold: 0.002    # [rad/s]
  settle_time: 0.5              # [s] continuously below threshold before averaging
  average_window: 1.0           # [s] samples averaged after settling
  timeout: 10.0                 # [s]
```

### `config/ur3e.yaml`
```yaml
robot_id: ur3e
base_frame: ur3e_base_link
flange_frame: ur3e_tool0        # D1: URDF FK frame, factory-calibrated, pendant-TCP-independent
joint_states_topic: /joint_states              # root, no namespace
joint_names: [ur3e_shoulder_pan_joint, ur3e_shoulder_lift_joint, ur3e_elbow_joint,
              ur3e_wrist_1_joint, ur3e_wrist_2_joint, ur3e_wrist_3_joint]   # used to MAP by name, not index
pose_source: tf
tip_offset: [0.0, 0.0, 0.10]    # PLACEHOLDER
estimate_tip_offset: true
tip_radius: 0.004
settle:
  joint_vel_threshold: 0.002
  settle_time: 0.5
  average_window: 1.0
  timeout: 10.0
```

### `config/plate.yaml`
```yaml
cone_half_angle_deg: 45.0
seats_nominal:                  # in plate frame [m], for sanity checks only
  p1_s1: [0.0, 0.0, 0.0]
  p1_s2: [0.150, 0.0, 0.0]
  p1_s3: [0.0, 0.150, 0.0]
min_captures_per_seat: 6
min_tilt_spread_deg: 15.0
max_condition_number: 1.0e4
reject_outliers: false
outlier_mad_k: 4.0
reference_robot: ur3e           # D2: robot A in T_A_B
```

Everything above is overridable from the launch/CLI command line.

> ⚠️ **Joint ordering:** FR3 order is fixed; the **UR `/joint_states` order is not pinned** (exploration §2). `kinematics.py`/`capture_node.py` must build the index map from the incoming `JointState.name`, never assume the `joint_names` list order. `joint_names` is used for validation/filtering and to record a canonical ordering, not for positional indexing.

---

## 6a. UR calibration pre-flight (operational requirement — exploration §4)

The UR3e is factory/per-unit calibrated **only when brought up via the repo launches** (`ur3e_handeye.launch.xml` / `dual_robots_check_handeye.launch.xml`, which load `my_robot_calibration.yaml`). If the UR is started with the stock driver launch, TF uses nominal DH (several-mm error) and `cone_calib` would silently inherit it. Therefore:

- The capture node **records `pose_source` and a note of the UR launch assumption** in every session file.
- 🟡 Operator must confirm, at bench time, that the UR driver logs **no kinematics-hash-mismatch warning** on bringup (live proof the calibration matches this physical UR3e).

---

## 7. Capture node (`capture_node.py`)

One node instance per robot, parameterised by `config/<robot_id>.yaml`. Subscribes to `joint_states_topic` and the TF tree; **publishes nothing that moves the robot**.

### 7.1 Trigger — DESIGN DECISION (flag at review)
The spec wants `~/capture` to carry `seat_id` in the request. A pure `ament_python` package cannot define a custom `.srv`. Two options:
- **(a) Recommended for the first cut:** terminal-prompt mode is primary (Enter = capture current seat, `n` = next seat, `u` = undo last, `q` = quit+save); the current seat is node state driven by `n` and the `plate.yaml` seat list. Expose `~/capture` as **`std_srvs/Trigger`** (captures for the *current* seat) and `~/next_seat`/`~/undo` as `Trigger` too. No new package; keeps "add a robot = one YAML."
- **(b) Faithful to the spec:** add a small `cone_calib_interfaces` (`ament_cmake`) with `Capture.srv` (`string seat_id → bool success, string message, float64 running_residual`). Second package, matches the literal spec. Requires its own entry in this doc's scope.

→ **Proceeding with (a)** unless you prefer (b).

### 7.2 Capture sequence
1. Wait until all joint velocities stay below `joint_vel_threshold` for `settle_time` (use published `velocity`; fall back to finite-difference of positions if absent — both robots publish velocity, so the fallback is belt-and-suspenders).
2. Average over `average_window`: mean joint positions; mean flange pose = position mean + **quaternion mean via the eigenvector method** (largest eigenvector of `Σ q qᵀ`, sign-canonicalised).
3. Time out with an error if the robot never settles.

### 7.3 Per-sample record (raw joints are essential — enable offline FK recompute)
`robot_id, seat_id, capture_index, timestamp, joint_names, mean_joint_positions, joint_position_std, mean_flange_pose {position, quaternion xyzw}, pose_source`.

### 7.4 Live feedback after each capture
- running capture count per seat;
- current tilt spread for that seat;
- once ≥ 3 captures exist for a seat, a running pivot residual using the current `tip_offset`.

### 7.5 Storage
Incremental append to a YAML/JSON file under a **timestamped session directory**, so a crash loses at most one capture. (Timestamp from ROS clock / wall clock in the node — allowed here; the *offline* solver stamps results from inputs.)

---

## 8. Offline solver (`ros2 run cone_calib solve`)

**Inputs:** one session file per robot, `plate.yaml`, optional `--reference-robot` (default from `plate.yaml` = `ur3e`), optional `--recompute-fk`.

**Pipeline:** pivot per robot → registration between robots → sanity checks.

**Results file (YAML):**
- `T_A_B` as translation, quaternion **xyzw**, RPY (deg), and a 4×4 matrix;
- estimated tip offsets per robot;
- seat centres per robot;
- all diagnostics + warnings (per-capture/per-seat/overall residuals, condition numbers, observability warnings, flagged outliers, CAD-distance deviations, two-route discrepancy, `world_to_base` cross-check);
- input file paths + a hash of the config used.

**Also emits** (so the result drops into the house ecosystem):
- a `static_transform_publisher` launch (`publish_calibration.launch.py`) **and/or** a params file;
- a **`static_tfs.yaml`-schema entry** (`{name: <parent>_to_<child>, parent, child, translation, quaternion xyzw, source, calibrated_on: 2026-10-07}`) ready to paste into `src/arms_data_collection/config/static_tfs.yaml`;
- a xacro `<joint>` snippet placing base B relative to base A.

**`--recompute-fk` (deferred, §6.4):** recompute flange poses from stored joint positions using the current URDF before solving. Requires a KDL/`urdf_parser` chain and **must be fed the UR `my_robot_calibration.yaml`** (else several-mm error). Not in the first cut; stubbed with a clear "not implemented" if the flag is passed.

---

## 9. Tests (`pytest`, ROS-free)

- **Synthetic round-trip:** random `T_A_B`, random tip offsets, random seat positions; captures with tilts within ±30°; Gaussian noise on positions (~0.1 mm) and rotations (~0.02°); assert recovery within noise-scaled tolerances.
- **Degenerate pivot:** rotations only about the tool axis → solver must warn/refuse to estimate `t`.
- **Collinear seats:** registration must raise.
- **Fixed-`t` mode:** matches joint estimation when `t` is exact.
- **Outlier flagging:** inject one corrupted capture → it is flagged.
- **Convention tests:** `T_A_B` direction (`x_A = T_A_B @ x_B`) and quaternion ordering (xyzw) — so a frame-inversion or quat-order bug cannot pass silently. Includes an explicit A=UR3e, B=FR3 orientation check.
- ament stubs (`flake8`/`pep257`/`copyright`) copied for parity; keep `cone_calib` ament-clean from day one (allowed, not required).

---

## 10. PR / commit plan

One commit per checklist step (`[PR-N] step X.Y: …`). `- [ ]` = code task; `- [ ] 🟡` = manual/physical, skip+flag.

### PR-1 — scaffold + SE(3)/registration math
- [ ] 1.1 Package skeleton (`setup.py`/`setup.cfg`/`package.xml` with numpy/scipy, resource marker, `__init__.py`, ament test stubs). Build green.
- [ ] 1.2 `se3.py`: homogeneous↔(t, quat xyzw), compose/invert, RPY, quaternion mean (eigenvector), Kabsch/Umeyama with reflection handling + degeneracy check.
- [ ] 1.3 `test_se3.py` + `test_conventions.py` (quat order, `T_A_B` direction, Umeyama recovery, collinear raise).

### PR-2 — pivot + registration + io
- [ ] 2.1 `pivot.py`: joint least-squares (`estimate_tip_offset` true/false), diagnostics, observability check, condition-number refuse, MAD outlier flagging.
- [ ] 2.2 `registration.py`: by-ID matching, Umeyama `T_A_B` (A=ur3e default), per-point residuals, CAD sanity, two-route discrepancy, `world_to_base` cross-check.
- [ ] 2.3 `io.py`: load/save raw session + results; `static_tfs.yaml`-schema emit; config hashing.
- [ ] 2.4 `test_pivot.py` + `test_registration.py` (synthetic round-trip, degenerate pivot, fixed-t, outlier, sanity, cross-check).

### PR-3 — ROS capture
- [ ] 3.1 `kinematics.py`: pose-source abstraction (TF lookup base→flange via `tf2_ros`; map joints by name; FK path stubbed).
- [ ] 3.2 `capture_node.py`: settle detection, averaging + quaternion mean, Trigger services (option a), terminal prompt mode, incremental timestamped save, live feedback.
- [ ] 3.3 `capture.launch.py` (robot_id → config), params overridable.

### PR-4 — solve CLI + publish + docs
- [ ] 4.1 `solve.py`: full pipeline, results YAML, emit launch/params + xacro snippet + static_tfs entry; `--reference-robot`; `--recompute-fk` guarded stub.
- [ ] 4.2 `publish_calibration.launch.py` (static_transform_publisher from results).
- [ ] 4.3 `README.md` (XYZW convention stated; procedure; UR-launch pre-flight; commands).

### PR-5 — bench validation
- [ ] 🟡 5.1 Dry-run capture on FR3 + UR3e (settle/averaging/live feedback), confirm live UR `/joint_states` ordering and no UR kinematics-hash warning.
- [ ] 🟡 5.2 Full capture (8–15 orientations × K seats × 2 robots), solve, compare `T_A_B` against `world_to_base` (linear ≤ 2 mm, angular ≤ 0.05 rad target).

---

## 11b. Implementation status (2026-10-07)

All software steps **done and verified**; only the 🟡 bench steps (PR-5) remain.

- PR-1…PR-4 implemented on `feature/cone-calib` (14 step-commits).
- Verified: `colcon build` clean; 30 ROS-free unit + integration tests pass; an
  end-to-end synthetic solve recovers `T_A_B` to ~0.6 mm / 0.09° under 0.1 mm +
  0.02° noise; `ament_flake8` green; capture node constructs/advertises/saves and
  tears down cleanly. `ament_pep257` shows ~33 docstring-style notices (D213 /
  section format) — left in the readable sibling style per the repo convention
  (siblings run red; not chased).
- An adversarial multi-agent review was run and **5 confirmed defects fixed**:
  (1) pivot `reject_outliers` KeyError when a whole seat is rejected (now dropped
  with a warning); (2) capture-node callback-group starvation (joint_states now
  in its own `ReentrantCallbackGroup` so settle works in service mode);
  (3) unlocked session-save race (all saves under the lock + unique temp file);
  (4) `>2` sessions silently dropped (now registers every pair into `results["pairs"]`
  with a warning); (5) cross-check false alarm under a flipped `--reference-robot`
  (now direction-agnostic). Each has a regression test where unit-testable.
- **Not pushed, no MR opened** — awaiting operator bench test + review.

## 11. Open items for review
- §7.1 trigger-service: option (a) Trigger + terminal (recommended) vs (b) `cone_calib_interfaces`.
- Real `tip_offset` / `tip_radius` values per probe (placeholders now).
- Real `plate.yaml` `seats_nominal` CAD coordinates and seat-ID scheme (`p<placement>_s<seat>`), and K.
- 🟡 Live-verification checklist in `exploration.md` §8 before PR-5.
- Confirm emitting the result **into** `arms_data_collection/static_tfs.yaml` stays out of scope here (cone_calib only *produces* the pasteable entry; editing that file is an `arms_data_collection` change needing its own doc).
