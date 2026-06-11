# Test Coverage Analysis — `smart_farm_spot`

_Analysis date: 2026-06-11_

## 1. Current state: effectively 0% automated coverage

| Signal | Finding |
|--------|---------|
| Unit/integration tests | **None.** No `test/` directory, no `test_*.py`, no `conftest.py`. |
| The one "test" file | `isaac/ros_bridge_test.py` is a manual ROS smoke script (live publishers/subscribers), not an automated test. |
| ament lint tests | Declared in `package.xml` (`ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`) but the conventional `test/test_flake8.py` / `test_pep257.py` / `test_copyright.py` files are **missing**, so `colcon test` runs nothing. |
| CI | No `.github/workflows`, no `tox.ini`, no `pytest.ini`. Nothing runs on push/PR. |
| `setup.py` | Has `tests_require=["pytest"]` but no `test_suite` and no tests to collect. |

~7,000 lines of Python (5,250 in `isaac/`, ~1,750 in ROS nodes) ship with no regression safety net.

## 2. Why this matters here

Most of the code can only run inside heavy external runtimes (Isaac Sim's `SimulationApp`, a live ROS 2 graph, Nav2 action servers, YOLO/ONNX models). That makes manual testing slow and rare — which is exactly the situation where small, fast, runtime-free unit tests pay off most. The good news: a meaningful slice of the logic is **pure** (geometry, parsing, image processing) and testable today with no simulator and no robot.

## 3. Highest-value targets (pure logic — testable now, no ROS/Isaac)

### 3.1 Pose / geometry helpers — _and the duplication that makes them risky_
`make_pose(x, y, yaw, frame)` is **copy-pasted in 4 files**:
`smart_farm_spot/waypoint_patrol.py`, `scenario_nav.py`, `h_drive.py`, `patrol.py`.
`load_waypoints` is duplicated in 2 files; `_yaw_from_quat` in 2 (`nav_policy_bridge.py`, `nav_policy_slam_bridge.py`); `_quat_mul`/`_yaw_quat`/`_visual_quat` in 2 (`isaac_sim_bridge.py`, `isaac/nav_bridge.py`). One of the copies subtly differs (`waypoint_patrol.make_pose` also sets `position.z = 0.0`).

> **Recommendation:** extract these into a shared, import-light module (e.g. `smart_farm_spot/geometry.py`) and unit-test it once. This kills the duplication and gives the tests a single home. Cases worth covering:
> - `make_pose`: yaw=0 → `(z=0, w=1)`; yaw=180° → `z≈1, w≈0`; yaw=90° → `z=w≈0.707`; `x/y/yaw` coerced to float; frame propagates.
> - `_yaw_from_quat`: round-trips with `_yaw_quat` across `[-π, π]`; handles the ±π wrap.
> - `_quat_mul`: identity element, known products, unit-norm preserved.

### 3.2 Cow-tail / cow-approach targeting math (`cow_tail_seek.py`, `nav_to_cow.py`)
These contain the project's most non-trivial math and the highest bug-surface, all currently untested:
- **Pixel→camera back-projection** (`cb_rgb`): `(u-cx)*d/fx`, `(v-cy)*d/fy`. Test with a synthetic intrinsics matrix and known points (principal point → origin ray, corner offsets).
- **`_sample_depth` rescaling** between RGB and depth resolutions (`int(u*dw/aw)`), plus the validity gate (`0.1 < d < 30.0`, `np.isfinite`). Test out-of-bounds, NaN/inf, and the boundary values.
- **"Behind the tail" goal** (`_maybe_send`): unit-vector from robot→tail, back-off `BEHIND` metres, yaw faces the tail; and the `RESEND_DIST` debounce that suppresses near-duplicate goals. The `dist < 1e-3` degenerate case is a divide-by-zero guard worth pinning.
- **Median stabilization** over the last `N_STABLE` samples.
- `nav_to_cow.tick`: the `APPROACH` offset point and the `d < 1e-3` fallback `(1,0)` direction.

These are pure once you factor them out of the ROS callbacks (today they're entangled with `self.tf_buf`, `self.model`, etc.) — see §6.

### 3.3 Thermal / mastitis hotspot vision (`wip/thermal_processor.py`)
The cleanest test target in the repo: `apply_fake_thermal`, `get_hotspot_bbox`, `draw_hotspot_overlay` take and return plain NumPy arrays (only `cv2`/`numpy`, no ROS). Worth testing because the `wip/` status means it's the most likely to silently rot:
- `apply_fake_thermal`: a synthetic image with a bright (>240) blob produces white pixels; the `>200` band produces the orange marker; output shape/dtype.
- `get_hotspot_bbox`: returns `None` below `min_area`; returns correct `x/y/w/h` and `cx/cy` centroid for a planted blob; picks the largest of several contours.
- `draw_hotspot_overlay`: does not mutate the input (`frame.copy()`), output shape unchanged.

### 3.4 YAML / waypoint & config loading (`load_waypoints`, `barn_map_server` metadata)
- `load_waypoints`: default `yaw=0.0` when key absent; empty/missing `waypoints` key → `[]`; malformed entry raises clearly.
- The shipped config files (`config/waypoints*.yaml`, `config/scenario_modes.yaml`, `maps/*.yaml`) deserve a **schema/smoke test** that loads each one and asserts required keys/types — catches a broken edit before it reaches the robot.
- `waypoint_patrol`'s fallback-to-`DEFAULT_WAYPOINTS`-on-load-failure path.

## 4. Second tier (needs ROS, but mockable)

The Nav2 client nodes (`patrol.py`, `scenario_nav.py`, `h_drive.py`, `waypoint_patrol.py`) are state machines worth testing with `rclpy` available and the action client mocked:
- **`patrol` / `scenario_nav` index + loop logic**: `_send_next` wrapping at end of list, `LOOPS` termination vs infinite, "look at next waypoint" yaw, goal-rejected → skip-to-next.
- **`scenario_nav` PATROL→SEEK preemption**: file-mtime change detection in `_watch_cow`, `RESEND_DIST` debounce, that SEEK suppresses further patrol goals.
- **`h_drive` status handling**: `status==4` success vs failure-but-continue.

These can largely be exercised by instantiating the node logic against a fake action client / fake clock without a live Nav2 server. Standard ROS practice is a `test/` dir run by `colcon test` with `launch_testing` for the genuinely integration-level paths.

## 5. Third tier — integration / Isaac (don't unit-test, smoke-test)

`isaac/*.py` (`scenario.py` 1,102 LOC, the bridges, spawners) depend on `SimulationApp` and can't be meaningfully unit-tested. Best ROI:
- Convert `ros_bridge_test.py` into a documented, optional integration smoke test (skipped when no ROS graph).
- Add an **import guard test**: every module imports cleanly (or fails gracefully) when Isaac/torch are absent. Several modules already guard imports (e.g. `thermal_processor.main`); a test would lock that behaviour in.

## 6. Cross-cutting recommendations

1. **Refactor for testability.** Pull the pure math out of ROS callbacks into free functions / a `geometry.py` + `vision.py`. This both removes the 4× `make_pose` duplication and makes §3 trivially testable.
2. **Stand up the harness.** Add `test/` with the three standard ament lint tests (flake8/pep257/copyright — already declared as deps but unused) plus a `tests/unit/` tree, and wire `colcon test` / `pytest`.
3. **Add CI.** A GitHub Actions workflow running `pytest` on the pure-logic tests (no Isaac/ROS needed for §3) would catch the majority of regressions on every PR.
4. **Pin environment-driven behaviour.** Lots of tunables come from env vars (`SF_BEHIND_TAIL`, `SF_SEARCH_WZ`, `SF_PATROL_LOOPS`, `SF_RESEND_DIST`, `BEHIND`, `APPROACH`). Tests should assert defaults and override parsing.

## 7. Suggested order of attack

1. Extract + test `geometry.py` (`make_pose`, quaternion/yaw helpers) — removes duplication, fastest win.
2. Test `thermal_processor.py` pure functions — zero refactor needed.
3. Test `load_waypoints` + a config-files schema smoke test.
4. Extract + test the cow-tail back-projection and "behind the tail" goal math.
5. Add the ament lint `test/` files and a minimal GitHub Actions `pytest` job.
6. Add mocked-action-client tests for the patrol/seek state machines.

Targeting items 1–4 alone would move the most defect-prone, runtime-free logic from 0% to solid coverage without standing up a simulator.
