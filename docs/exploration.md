# cone_calib — Step 0 workspace exploration

**Date:** 2026-10-07
**Branch:** `feature/cone-calib` (cut from `dev`, in sync with `origin/dev`)
**Author:** Claude (read-only reconnaissance) — no files outside this doc were modified.
**Method:** 5 parallel read-only investigations over `$ros2_workspace`, then 3 of the load-bearing
claims (frame names, UR factory calibration, existing-calibration conventions) were
independently re-verified from primary evidence by separate agents. All claims below carry a
confidence level; the three re-verified topics also carry a verification verdict.

> This is the mandatory *explore-before-code* record for the proposed `cone_calib` package
> (base-to-base calibration of the FR3 + UR3e via conical seats). **No implementation code has
> been written.** The next step, per the project design-doc-first rule, is a date-prepended
> design doc in `docs/` agreed with the operator. Open decisions are listed in §9.

---

## 0. TL;DR — verdicts

| Question | Answer | Confidence |
|---|---|---|
| FR3 joint-state topic | `/fr3/franka_state_publisher/joint_states` (namespace **`fr3`**, *not* `/franka/...`) | high |
| UR3e joint-state topic | `/joint_states` (root, no namespace) | high |
| Velocity published? | **Yes, both robots** — no finite-differencing needed (fallback still worth keeping) | high |
| FR3 base / flange frame (URDF) | `fr3_link0` / `fr3_link8` | high (verified) |
| UR3e base / flange frame (URDF) | `ur3e_base_link` / `ur3e_tool0` (prefix `ur3e_`); driver TCP = `ur3e_tool0_controller` | high (verified) |
| Flange pose source | **TF via `robot_state_publisher`** for both (robot-agnostic). FR3 also exposes a custom `O_T_EE` topic, but that is the *Desk-configured EE* (≈ spatula tip), **not** the flange — do not use it here. | high (verified) |
| UR3e kinematics | **FACTORY / per-unit calibrated** (real `ur_calibration` DH deltas reach `robot_description`), *provided the robot is brought up via the repo launches* | high (verified, see §4 caveat) |
| Existing base-to-base method | Two eye-on-base hand-eyes (`cv2.calibrateHandEye`, Tsai-Lenz) composed through a shared OptiTrack frame. **No Kabsch/Umeyama/Procrustes point-set registration exists in first-party code** → the spec's pivot+registration math is net-new. | high (verified) |
| First-party rotation convention | `scipy.spatial.transform.Rotation`, quaternion **XYZW** everywhere (only vendored `easy_handeye2` uses WXYZ) | high (verified) |
| Package type for `cone_calib` | `ament_python`, lives at `src/cone_calib/` | high |

---

## 1. Build & package conventions

- **All in-house Python packages are `ament_python`** (`arms_data_collection`, `arms_human_data_collection`, `ur3e_control`). `ament_cmake` is used only for interface/message packages (`*_interfaces`, `sam3_ros2_interfaces`). A pure-Python package *cannot* generate custom msgs/srvs — if `cone_calib` ever needs them, they go in a separate `cone_calib_interfaces` (`ament_cmake`). The spec does not require custom interfaces (a plain `std_srvs`/custom-free trigger service suffices), so a single `ament_python` package is expected.
- **Cleanest template to copy:** `src/arms_data_collection` (minimal). Most complete lint/test template: `src/arms_human_data_collection`.
- **Layout a new package needs:** `src/cone_calib/cone_calib/` (module, `__init__.py`), `resource/cone_calib` (0-byte marker, first `data_files` entry), `setup.py`, `setup.cfg`, `package.xml` with `<export><build_type>ament_python</build_type></export>`, plus `config/`, `launch/`, `docs/`, `test/`.
- **`setup.py` data_files pattern** (from `arms_data_collection/setup.py`): installs the resource marker, `package.xml`, and `glob('launch/*launch.[pxy][yma]*')` (matches **both** `.launch.py` and `.launch.xml`) + `glob('config/*.yaml')`. `console_scripts` entries as `'<exe> = cone_calib.<module>:main'`.
- **Dependencies:** ROS deps (`rclpy`, `tf2_ros`, `tf2_ros_py`, `geometry_msgs`, `sensor_msgs`, `std_srvs`) declared in `package.xml` as `<exec_depend>`. **`numpy`/`scipy` are NOT declared anywhere in-house** (treated as system/transitive); `install_requires=['setuptools']`. `setup.cfg` only sets `script_dir`/`install_scripts` to `$base/lib/cone_calib`. (We may choose to declare numpy/scipy for correctness — flag in design doc; matching the sibling convention means leaving them out.)
- **Lint:** standard three ament stubs under `test/` (`test_flake8.py`, `test_pep257.py`, `test_copyright.py`; copyright one is `@pytest.mark.skip`). No `[flake8]` override in-house (ament default line length 99). The flake8/pep257 gate is **pre-existing RED** in `arms_human_data_collection` by convention (memory `feedback_arms_human_data_collection_lint_red`); the rule is "no worse than the sibling," not "chase green." Because `cone_calib` is brand-new we *can* keep it ament-clean from day one — allowed, not required.
- **Pure-math unit tests** use plain `pytest` (`def test_*` + `assert`, importing the module directly) — precedent `src/ros2launch_gui/test/test_ansi_to_html.py`. This is exactly how the ROS-free `pivot.py`/`registration.py`/`se3.py` modules in the spec should be tested.
- **Build:** from the workspace root. Single package: `cd $ros2_workspace && colcon build --packages-select cone_calib --symlink-install && source install/setup.bash`. `CLAUDE.md` "Build & test" is still "Not yet implemented," so there is no committed canonical test-gate command; `colcon test` / `pytest` usage is inferred.

---

## 2. Joint-state publication

### FR3 (Franka) — vendored in `src/rli_ws_base2` (git submodule)
- Primary topic: **`/fr3/franka_state_publisher/joint_states`** — published by a **custom** controller `franka_ros2_controllers/FrankaStatePublisher` (not the stock `joint_state_broadcaster`, which is commented out for the FR3). Source: `src/rli_ws_base2/franka_ros2_controllers/src/franka_state_publisher.cpp:70,110-113`; config `src/rli_ws_base2/py_panda2/config/epfl_robot_controllers.yaml:10-11,114-121`.
- Joint names/order (fixed, verbatim from config): `fr3_joint1 … fr3_joint7` (7 arm joints, no gripper unless `load_gripper:=true`).
- Fields: **position + velocity + effort** all populated (effort = *desired* effort). Rate **250 Hz**.
- There is also an aggregated **`/fr3/joint_states`** (a `joint_state_publisher` node merging `source_list`, 30 Hz) that feeds `robot_state_publisher`/TF/RViz — it also carries velocity.
- Repo precedent for the primary topic as default: `src/arms_data_collection/arms_data_collection/fr3_joint_limit_gui.py:49`.
- FR3 launch: `src/rli_ws_base2/py_panda2/launch/epfl_bimanual_robot.launch.py` (and `epfl_single_robot.launch.py`).

### UR3e (Universal Robots) — NOT vendored; from the `/opt/ros/jazzy` overlay
- Topic: **`/joint_states`** (root, no `/ur` namespace) — published by stock `joint_state_broadcaster`. Source: `/opt/ros/jazzy/share/ur_robot_driver/launch/ur_control.launch.py:70-80,179-190`.
- Joint names: prefixed `ur3e_` in the repo launches (`tf_prefix=ur3e_`): `ur3e_shoulder_pan_joint, ur3e_shoulder_lift_joint, ur3e_elbow_joint, ur3e_wrist_1_joint, ur3e_wrist_2_joint, ur3e_wrist_3_joint`.
- Fields: **position + velocity + effort** populated.
- ⚠️ **Order is NOT pinned** (`joint_state_broadcaster` has no `joints` list → publishes "all available interfaces"). URDF declaration order is *likely* (shoulder_pan…wrist_3) but **not contractually guaranteed**. **Design implication: map joints by `msg.name`, never by a hardcoded index.** Confirm live with `ros2 topic echo /joint_states --once`.

**Both robots publish velocity natively**, so settle-detection can read `JointState.velocity`; keep the finite-difference-of-position fallback the spec already specifies for robustness.

---

## 3. Flange / end-effector pose source + exact frame names (VERIFIED — CONFIRMED)

### FR3
- TF is published by `robot_state_publisher` under ROS namespace **`fr3`**; TF frame *ids* are the link names (not namespaced): base **`fr3_link0`**, flange **`fr3_link8`**. There is also a root link literally named **`base`** above `fr3_link0` (`world → base → fr3_link0`; `world→base` is an identity `static_transform_publisher` in the single-robot launch). Evidence: `src/rli_ws_base2/franka_ros2/franka_description/robots/common/franka_arm.xacro:146-152,23-33`; launch `epfl_single_robot.launch.py:99,150-153`.
- `hand:=false` by default ⇒ **`fr3_hand` / `fr3_hand_tcp` are NOT in the live tree**.
- A **custom** `O_T_EE` topic exists: `/fr3/franka_state_publisher/state` (`franka_ros2_controllers_msgs/RobotState`, field `model.ee_pose` = 16 floats, column-major 4×4 = libfranka `getPoseMatrix(franka::Frame::kEndEffector)`). This is the **Desk-configured EE frame (≈ the mounted spatula tip), not `fr3_link8`** — `static_tfs.yaml`'s `link8_to_spatula` even carries the banner "IF THIS VALUE CHANGES, UPDATE FRANKA DESK." **For cone_calib the probe is a separate sphere-tipped tool, so we want the flange `fr3_link8` via TF and express `tip_offset` in it — the `O_T_EE` topic would be actively wrong here.** The stock `franka_robot_state_broadcaster`/`franka_msgs/FrankaState` is **declared but never spawned** in the EPFL launches, so do not rely on it.

### UR3e
- TF via the binary `ur_robot_driver` + `robot_state_publisher`, `tf_prefix=ur3e_`. Frame chain (URDF): `ur3e_base_link` (ROS conv.) / `ur3e_base` (UR rotated conv.) … `ur3e_wrist_3_link → ur3e_flange → ur3e_tool0`. Evidence: `/opt/ros/jazzy/share/ur_description/urdf/ur_macro.xacro:86,377,235,390,398`.
- The driver **additionally** publishes `ur3e_tool0_controller` — the controller-reported TCP (= `ur3e_tool0` + the TCP configured on the UR pendant; repo note says +40 mm in the hand-eye setup). This frame is **not** in the URDF; it is what the existing hand-eye pipeline used as the effector frame. **`ur3e_tool0` (URDF FK, factory-calibrated, deterministic) vs `ur3e_tool0_controller` (pendant-TCP-dependent) is an open decision — see §9.**

### FK utilities
- The **only** in-workspace FK is a KDL parser (`PyTinyURDFParser.KDLRobot`) inside `py_panda2` `OfflineModel` (`src/rli_ws_base2/py_panda2/py_panda2/models.py`), and it is **currently commented out/disabled** (`robot.py` uses `ROSModel` instead). No Pinocchio / PyKDL / pytransform3d in active use.
- **Implication for the spec's optional `--recompute-fk`:** there is no ready FK to call. Offline FK-from-joints would need a URDF + KDL/`urdf_parser` chain *and must be fed the UR factory calibration* (else several-mm error). Recommend: **`pose_source: tf` is the primary path for the first cut; `--recompute-fk` is future/optional** and, if built, must consume `my_robot_calibration.yaml` for the UR.

---

## 4. UR3e kinematics calibration — CRITICAL (VERIFIED — headline CONFIRMED, mechanism corrected)

**Verdict: the UR3e is FACTORY / per-unit calibrated, not nominal DH** — decisive and good for a calibration package. Both `ur3e_control` launches override the driver's nominal default with a real `ur_calibration` output:

- `src/ur3e_control/launch/ur3e_handeye.launch.xml:28,35` and `dual_robots_check_handeye.launch.xml:4,32` set `ur_kinematics` → `/home/swisscat/my_robot_calibration.yaml` and pass it as `kinematics_params_file`.
- That file is **byte-identical** (md5 `836e077e6943ad009b99e552580bc379`) to the in-repo copy `src/ur3e_control/calib/my_robot_calibration.yaml`.
- It contains **genuine non-zero per-unit deltas** vs nominal (e.g. `upper_arm.x=-8.53e-05` [nom 0], `upper_arm.roll=1.570598874` [nom 1.570796327, ≈ −197 µrad], `wrist_1.y=-0.000676` [nom 0], `wrist_3.y=0.09185` [nom 0.0921]). Hash `calib_15738745451911568382` ≠ nominal `calib_16756443741236045476`. Nominal reference: `/opt/ros/jazzy/share/ur_description/config/ur3e/default_kinematics.yaml`.

**⚠️ Caveats that matter for cone_calib:**
1. **The factory calibration only reaches TF if the robot is brought up via the repo launches.** If someone starts the UR with the stock `ur_robot_driver` launch (nominal default), TF would be several-mm off and `cone_calib` (reading TF) would silently inherit that error. → *Design/operational requirement: record `pose_source`, and ideally the kinematics hash, with every session; require the repo launch; at bench time confirm the driver logs **no** kinematics-hash-mismatch warning (the live proof the file matches this physical robot).*
2. **The calibration reaches the URDF via a fragile launch-config scope-leak, not explicit argument forwarding** (verifier correction). `ur_control.launch.py` (ur_robot_driver 3.6.0) has *no* `kinematics` references and forwards only `robot_ip`/`ur_type` to `ur_rsp.launch.py`; the real value survives because `IncludeLaunchDescription` leaks `SetLaunchConfiguration` into the shared context and `ur_rsp`'s `DeclareLaunchArgument` default only applies when the key is absent. End result is correct (real DH in `robot_description`), but the chain is non-obvious and brittle — do not "fix" the repo launches casually.
3. **Absolute-home-path dependency:** both launches default to `/home/swisscat/my_robot_calibration.yaml` (outside the repo). A fresh clone on another machine has no fallback. The in-repo copy `src/ur3e_control/calib/my_robot_calibration.yaml` is identical today. (Any change to the launch default is a `ur3e_control` concern and needs its own design doc — out of scope for `cone_calib`.)
4. The repo's own `ur3e_with_hande.urdf.xacro:10-11` defaults to a **broken** `config/` path (dir doesn't exist; file is in `calib/`), but that xacro feeds only the visualization-only `urdf_viewer.launch.py`, **not** the driver bringup.
5. The file has **no capture-date annotation** (violates the workspace calibration-date convention, memory `feedback_calibration_dates`); file mtime is 2026-01-09 but that is not authoritative provenance.

---

## 5. Existing calibration code & conventions to reuse (VERIFIED — CONFIRMED)

**There is NO Kabsch/Procrustes/Umeyama/SVD point-set registration for robot base-to-base in first-party code.** The only `align_vectors` call aligns a single surface normal (`tools/decomposition_class_original.py:839`). → **The spec's pivot calibration + Kabsch/Umeyama registration is net-new; plan to implement it in `se3.py`/`pivot.py`/`registration.py`.**

How base-to-base is done *today* (useful as an **independent cross-check** of the cone result, not something to reuse wholesale):
- Two **eye-on-base** hand-eyes (FR3 and UR3e, each vs a shared OptiTrack `mocap_base`) via vendored `easy_handeye2` → `cv2.calibrateHandEye`, default **Tsai-Lenz** (`src/rli_ws_base2/easy_handeye2/.../handeye_calibration_backend_opencv.py:83`).
- Composed through the mocap frame by `DualRobotsTfComposer`, which **inverts** each hand-eye and publishes `virtual_mocap_base → base` (FR3) and `virtual_mocap_base → world` (UR = `ur3e_base_link`). The final `ur3e_base_link → base` (= `world_to_base`) is read off TF with `print_tf_precise` and pasted into `static_tfs.yaml`.
- **#1 documented failure mode:** `dual_robots_tf_composer.py:73-98` carries its *own* hardcoded duplicate copies of both hand-eye literals that must be hand-synced to the YAML (currently stale, confirming they are independent copies).

**Conventions & utilities to REUSE (so cone_calib drops into the existing ecosystem):**
- **Quaternion order: XYZW** (`scipy.spatial.transform.Rotation`, `R.from_quat([x,y,z,w])` / `R.as_quat()`). Uniform across all first-party code. Only vendored `easy_handeye2` uses WXYZ (transforms3d) — convert explicitly at that seam (pattern: `run_spatula_pose_export_world_mocap_fix.py:530-537`). **cone_calib must state XYZW in the README and use it everywhere.**
- **`static_tfs.yaml` schema** (`src/arms_data_collection/config/static_tfs.yaml`): top-level `transforms:` list, each `{name: <parent>_to_<child>, parent, child, translation:[x,y,z] (m), quaternion:[x,y,z,w] XYZW, source, calibrated_on: YYYY-MM-DD}`. Emit the cone_calib result in exactly this shape so it can drop into the single source of truth.
- **Validation to reuse/mirror:** `src/arms_data_collection/arms_data_collection/tf_static_publisher.py:25-107` (required fields, quat norm within 1e-3, no duplicate names/edges, `parent != child`).
- **`invert_transform()`** — `src/ur3e_control/ur3e_control/dual_robots_tf_composer.py:14` (scipy, XYZW). Reuse rather than re-derive. (It lives in another package; if we want it *inside* cone_calib we copy the few lines rather than cross-import — see scope rule.)
- **Acceptance metric** — `print_tf_precise.py:73-82`: `linear = ||t||` (m), `angular = 2·acos(|q_w|)` (rad); documented thresholds **linear ≤ 2 mm, angular ≤ 0.05 rad** (a manual eyeball check today, not an auto gate). Reuse these as cone_calib's own residual/acceptance report thresholds.
- **Existing `world_to_base` value** (cross-check target): parent `world`(=`ur3e_base_link`), child `base`(FR3 root, parent of `fr3_link0`), translation `[-0.154826244, 0.795879495, -0.001069670]`, quaternion `[-0.001543266, -0.006411778, 0.002305206, 0.999975596]`, `calibrated_on 2026-06-01` (the UR `.calib` was re-done 2026-10-05, so the live value may be newer). cone_calib's `T_A_B` can be compared to this **modulo the `base`↔`fr3_link0` and `world`↔`ur3e_base_link` aliases** — a great independent validation, not an equality to assume.

---

## 6. Consolidated key-values (seeds for `config/fr3.yaml`, `config/ur3e.yaml`)

| Param | FR3 | UR3e |
|---|---|---|
| `robot_id` | `fr3` | `ur3e` |
| `base_frame` | `fr3_link0` | `ur3e_base_link` |
| `flange_frame` | `fr3_link8` | `ur3e_tool0` *(or `ur3e_tool0_controller` — §9)* |
| `joint_states_topic` | `/fr3/franka_state_publisher/joint_states` | `/joint_states` |
| `joint_names` | `fr3_joint1..fr3_joint7` (fixed order) | `ur3e_shoulder_pan_joint, ur3e_shoulder_lift_joint, ur3e_elbow_joint, ur3e_wrist_1_joint, ur3e_wrist_2_joint, ur3e_wrist_3_joint` (**map by name, not index**) |
| `pose_source` | `tf` | `tf` |
| velocity available | yes (250 Hz) | yes |

Repo launch entrypoints for context: FR3 `src/rli_ws_base2/py_panda2/launch/epfl_bimanual_robot.launch.py`; UR3e `src/ur3e_control/launch/ur3e_handeye.launch.xml` (+ `dual_robots_check_handeye.launch.xml`). UR `robot_ip` default `192.168.56.101`; FR3 `robot_ip` hardcoded `192.168.2.33`.

---

## 7. How the findings change the spec's assumptions

| Spec assumption | Reality | Action |
|---|---|---|
| `fr3.yaml: joint_states_topic: /franka/joint_states` | It's `/fr3/franka_state_publisher/joint_states` | Update default |
| `fr3.yaml: flange_frame: fr3_link8` | Correct ✅ (and `base_frame: fr3_link0` ✅) | Keep |
| (UR flange frame unspecified) | `ur3e_tool0` vs `ur3e_tool0_controller` | **Decide (§9)** |
| joint order from an explicit list | FR3 fixed; **UR order not pinned** | Map by `msg.name` at runtime; keep the explicit list only as a validation/filter |
| velocities may be unavailable → finite-diff | Both publish velocity | Prefer published velocity; keep finite-diff fallback |
| `pose_source: tf | fk`, FR3 could use `O_T_EE` | `O_T_EE` is the Desk EE (spatula), not the flange | **Use `tf` → flange for both**; don't use `O_T_EE` |
| `--recompute-fk` optional | No active FK in workspace; UR needs factory calib fed in | Make `tf` primary; `--recompute-fk` future, must load `my_robot_calibration.yaml` |
| reuse earlier Kabsch/Procrustes work | None exists in first-party code | Implement fresh in `se3.py`/`registration.py` |
| output format | `static_tfs.yaml` schema is the house standard (XYZW, `calibrated_on`) | Emit results in that schema + the `<parent>_to_<child>` naming |

Everything else in the spec (per-robot pivot joint least-squares, observability/condition-number guards, outlier MAD flagging, Umeyama with reflection handling, ≥3 non-collinear seats, CAD sanity checks, capture node with settle/averaging/quaternion-mean, offline solver, synthetic round-trip tests) stands as written and is compatible with this workspace.

---

## 8. Open questions / live-verification checklist (do on the running robots)

- [ ] 🟡 `ros2 topic echo /joint_states --once` (UR) — capture the **actual** joint name ordering.
- [ ] 🟡 `ros2 topic echo /fr3/franka_state_publisher/joint_states --once` (FR3) — confirm names/order.
- [ ] 🟡 Confirm the UR driver bringup logs **no** kinematics-hash-mismatch warning (live proof the factory calib matches this physical UR3e).
- [ ] 🟡 Confirm both robots' TF trees are up (`fr3_link0→fr3_link8`, `ur3e_base_link→ur3e_tool0`) when the capture node runs, and whether FR3 and UR are ever in the same ROS graph (FR3 under `/fr3`, UR at root — no `/joint_states` collision expected, but verify).
- [ ] Physical `my_robot_calibration.yaml` provenance/date (no capture-date in the file; mtime 2026-01-09 only).
- [ ] Whether the deployment uses `epfl_single_robot` vs `epfl_bimanual` (both set `arm_id=fr3`, identical frames; single adds a `world→base` static TF the bimanual omits).

---

## 9. Decisions (RESOLVED 2026-10-07)

1. **UR3e flange frame:** ✅ **`ur3e_tool0`** (URDF FK, factory-calibrated, deterministic, pendant-TCP-independent). `tip_offset` is defined relative to it.
2. **Reference robot A** for the reported `T_A_B` (= pose of base B in base A): ✅ **UR3e (`ur3e_base_link`) as A** ⇒ `T_A_B` = FR3 base (`fr3_link0`) expressed in the UR3e base — directly comparable to the existing `world_to_base` (= `T_ur3e_base_link ← base`). A `--reference-robot` flag keeps both directions derivable.
3. **Package name** ✅ `cone_calib`.
4. ✅ **Declare `numpy`/`scipy`** in `package.xml` (stricter than the undeclared-sibling convention, but more correct for a package that fundamentally depends on them).

Design doc: `src/cone_calib/docs/2026-10-07_CONE_CALIB_DESIGN.md`.
