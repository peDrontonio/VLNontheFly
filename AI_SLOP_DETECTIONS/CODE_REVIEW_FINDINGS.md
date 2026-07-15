# VLNontheFly Code Review Findings
**Basis:** IMAV 2026 Reviewer Feedback (imav2026_reviewer_feedback.md)  
**Date:** 2026-07-13  
**Scope:** Full codebase analysis for inconsistencies with paper claims and reviewer concerns

---

## Critical Issues

### 1. **MoCAP Dependency Contradicts "Fully Onboard" Claim** ⚠️
- **Location:** [planner_wrapper/launch/ego_raptor.launch.py](../planner_wrapper/launch/ego_raptor.launch.py#L20-22)
- **Issue:** Paper claims "Fully Onboard Vision-Language Navigation Stack" but implementation hardcodes `with_optitrack:=false` as default. All hardware tests use OptiTrack pose estimation.
- **Reviewer Concern:** Reviewer 1 states: *"The paper is supposed to present a 'Fully Onboard Vision-Language Navigation Stack,' but the OptiTrack PrimeX 41 motion-capture system provides pose estimates to the vehicle."*
- **Code Evidence:**
  ```python
  # Default: with_optitrack:=false, but HARDWARE_TESTS.md requires OptiTrack enabled
  DeclareLaunchArgument('with_optitrack', default_value='false', ...)
  ```
- **HARDWARE_TESTS.md Reality Check:** [HARDWARE_TESTS.md:76-78](../ego-planner-swarm/HARDWARE_TESTS.md#L76-78) shows Stage 0-3 all require `with_optitrack:=true` for working flights
- **Gap:** No onboard VIO/SLAM fallback tested. VIO option mentioned in review but not implemented.
- **Severity:** HIGH - contradicts paper's core claim

---

### 2. **ESDF Unexplained Yet Central to Planner** ⚠️
- **Location:** Review feedback mentions ESDF but term never explained in code or docs
- **Issue:** Reviewer 3 states: *"The use of EGO-planner, an ESDF-free planner"* — but no documentation explains:
  - What ESDF is (Euclidean Signed Distance Field)
  - Why EGO-planner being "ESDF-free" matters
  - How this affects obstacle representation vs. a grid map
- **Current State:** 
  - Grid map used throughout (planner_wrapper)
  - No mention of ESDF in code, launch files, or configuration
  - Only mentioned in reviewer feedback
- **Gap:** Paper and code lack clarity on planner's collision-checking mechanism
- **Severity:** MEDIUM - theoretical gap, not a bug

---

### 3. **Coarse 3×3 Grid Vulnerability Not Addressed** ⚠️
- **Location:** [edgellm_vlm_ros/config/vlm_node.yaml](../edgellm_vlm_ros/config/vlm_node.yaml#L59) and [edgellm_vlm_ros/scripts/vlm_region_gate.py](../edgellm_vlm_ros/scripts/vlm_region_gate.py#L243-244)
- **Issue:** Reviewer 3 identifies specific failure modes:
  - **Prediction Oscillation:** VLM alternates between adjacent cells when target spans multiple cells
  - **Edge Clipping:** Selecting boundary cell causes depth to be calculated from background
  - **Close-Range Failure:** When drone approaches target, 3×3 grid becomes too coarse
- **Code Evidence:**
  ```python
  self.grid_cols = int(self.declare_parameter("grid_cols", 3).value)
  self.grid_rows = int(self.declare_parameter("grid_rows", 3).value)
  ```
- **Partial Mitigations DO Exist in Code** (correction after recheck — the code is better than the paper explains):
  - `cooldown_s: 2.0` — goals published at most every 2 s, which damps (but does not eliminate) oscillation between adjacent cells ([region_gate.yaml:72](../edgellm_vlm_ros/config/region_gate.yaml#L72))
  - `standoff_m: 0.8` — in target mode the drone stops 0.8 m short of the target surface, so it never enters the close range where the object spans multiple cells ([vlm_region_gate.py:198-209](../edgellm_vlm_ros/scripts/vlm_region_gate.py#L198-209), `apply_standoff`)
  - `max_goal_distance_m: 1.5` — each hop is clamped, limiting the damage of any single bad cell pick
  - `min_confidence: 0.50` + median depth over the whole cell (not a single pixel) — reduces edge-clipping impact, since a boundary cell's median is dominated by whichever surface fills most of it
- **What is genuinely missing:**
  - No explicit hysteresis (previous-region stickiness); cooldown limits rate, not direction flapping
  - No detection of the specific edge-clip case (bimodal depth distribution in a cell)
  - No discussion in the paper of why 3×3 vs. pixel-level segmentation (SAM)
- **Success Radius Question:** Reviewer asks: *"Was the 20 cm success radius explicitly chosen as a threshold to stop the test before these grid-overlapping and edge-clipping issues could manifest?"*
  - No 0.2 m threshold exists in this code. The de-facto stop mechanism is `standoff_m: 0.8` — the drone deliberately halts 0.8 m before the target, which is exactly the regime boundary the reviewer suspects. This is a legitimate design choice but should be stated openly in the rebuttal: yes, the pipeline stops before the close-range regime, by design, via the standoff.
- **Severity:** MEDIUM (downgraded after recheck) - mitigations exist but are undocumented; hysteresis and edge-clip detection are still absent

---

### 4. **Safety Supervisor Implementation Underspecified** ⚠️
- **Location:** [edgellm_vlm_ros/scripts/vlm_nav_supervisor.py](../edgellm_vlm_ros/scripts/vlm_nav_supervisor.py) and [edgellm_vlm_ros/config/nav_supervisor.yaml](../edgellm_vlm_ros/config/nav_supervisor.yaml)
- **Reviewer Concern:** Reviewer 1: *"The safety supervisor is presented as one of the main contributions, but its implementation and rules are not explained in sufficient detail."*
- **Code Reality (corrected after recheck):** the "safety layer" is actually **distributed across three places**, and the supervisor is only one of them:
  1. **Gates** ([vlm_point_gate.py](../edgellm_vlm_ros/scripts/vlm_point_gate.py), [vlm_region_gate.py](../edgellm_vlm_ros/scripts/vlm_region_gate.py)) — this is where the physical validation lives: median depth over a window, `min_clearance_m`, confidence threshold, staleness checks on result/depth/pose (`max_result_age_s`, `max_depth_age_s`, `max_pose_age_s`), hop clamping (`max_goal_distance_m`), cooldown
  2. **Supervisor** ([vlm_nav_supervisor.py](../edgellm_vlm_ros/scripts/vlm_nav_supervisor.py)) — a mode FSM (POINT_NAV → ALTITUDE_ADJUST → SETTLE → HOLD) that counts gate rejections and switches VLM prompt modes; it does no physical validation itself
  3. **EGO-planner goal gate** (external package, configured per [HARDWARE_TESTS.md §Stage 3 prep](../ego-planner-swarm/HARDWARE_TESTS.md)) — allowed box + tripod keep-out cylinders, applied to every goal regardless of source
- **The actual problem** is therefore not that safety checks are missing — many exist — but that:
  - The paper apparently presents "the safety supervisor" as one contribution, while the real safety logic is spread over gates + supervisor + planner-side goal gate, none of which is fully specified in the paper (per Reviewer 1)
  - Velocity/acceleration limits are enforced only planner-side (EGO-planner `max_vel`/`max_acc`), not re-checked downstream
  - There is no single document enumerating all rules — a reviewer cannot reconstruct the safety envelope from the paper
- **Config Parameters:** [nav_supervisor.yaml](../edgellm_vlm_ros/config/nav_supervisor.yaml)
  ```yaml
  point_rejection_limit: 3        # arbitrary count
  primitive_rejection_limit: 2    # arbitrary count
  max_proposal_age_s: 3.0         # staleness check only
  settle_s: 2.0                    # no physical validation
  ```
- **Severity:** MEDIUM - supervisor is a gate controller, not a safety layer; nomenclature is misleading

---

### 5. **Hardcoded Paths Make Code Non-Portable** 🔴
- **Locations:**
  - [edgellm_vlm_ros/src/vlm_node.cpp:330-335](../edgellm_vlm_ros/src/vlm_node.cpp#L330-335)
  - [edgellm_vlm_ros/config/vlm_node.yaml:7,10,13](../edgellm_vlm_ros/config/vlm_node.yaml#L7-13)
  - [edgellm_vlm_ros/launch/d435i_vlm.launch.py](../edgellm_vlm_ros/launch/d435i_vlm.launch.py)
- **Issue:** Paths to model engines hardcoded to `/home/orin/imav/`:
  ```yaml
  engine_dir: "/home/orin/imav/tensorrt-edgellm-workspace/Qwen3.5-2B/int4_awq/engines/llm"
  multimodal_engine_dir: "/home/orin/imav/tensorrt-edgellm-workspace/Qwen3.5-2B/int4_awq/engines/visual"
  plugin_path: "/home/orin/TensorRT-Edge-LLM/build/libNvInfer_edgellm_plugin.so"
  ```
- **Impact:** Code fails to run on any other system without manual editing
- **Reproducibility:** Blocks reproducibility and makes code unsuitable for review/deployment
- **Fix:** Use ROS package paths or environment variables
- **Severity:** HIGH - blocks reproducibility

---

### 6. **Prompting Strategy Lacks Documentation** ⚠️
- **Location:** [edgellm_vlm_ros/config/vlm_node.yaml](../edgellm_vlm_ros/config/vlm_node.yaml#L43-62)
- **Reviewer Concern:** Reviewer 3: *"Crucial implementation details regarding the VLM are missing. It is unclear how the 3×3 grid is used querying the VLM."*
- **Current State:**
  - **Point mode:** Expects pixel coordinates `{u, v, confidence}`
  - **Region mode:** Expects grid cell name `{region, confidence}`
  - **Primitive mode:** Expects movement command `{primitive, distance_m, confidence}`
  - Prompts are detailed but **mechanism of grid overlay is never explained**
- **Missing Documentation:**
  - Is the grid superimposed on the image before query, or is it a post-process over raw text?
  - How are the 9 region names mapped to pixel coordinates?
  - What happens if VLM returns pixel coords outside the image bounds?
  - How is confidence interpreted when VLM gives multiple "plausible" answers?
- **Code Evidence:** [vlm_region_gate.py:366-374](../edgellm_vlm_ros/scripts/vlm_region_gate.py#L366-374)
  ```python
  def _decide_cell(self) -> CellDecision:
      # Depth-validates the VLM's grid cell choice
      # But how does VLM know the grid? Not documented.
  ```
- **Severity:** MEDIUM - affects reproducibility and ablation studies

---

### 7. **Code Defaults vs. Config Values Diverge (temperature, rate)** ⚠️
- **Location:** [edgellm_vlm_ros/config/vlm_node.yaml:19-26](../edgellm_vlm_ros/config/vlm_node.yaml#L19-26) vs. [edgellm_vlm_ros/src/vlm_node.cpp:337-340](../edgellm_vlm_ros/src/vlm_node.cpp#L337-340)
- **Issue (corrected after recheck):** The C++ parameter defaults and the shipped YAML config disagree:
  | Parameter | C++ default (vlm_node.cpp) | YAML config (vlm_node.yaml) |
  |---|---|---|
  | `temperature` | 0.2 | 0.5 |
  | `fixed_rate_hz` | 1.0 | 0.5 |
  | `max_generate_length` | 96 | 48 |
  | `publish_partials` | true | false |
- **Important nuance:** the YAML's temperature 0.5 is **intentional and documented** in the file itself: *"Kept low for stable JSON, but not so low the model freezes on one answer."* (A too-low temperature on a small quantized VLM can collapse to always answering CENTER — the code comment at [vlm_node.cpp:378-379](../edgellm_vlm_ros/src/vlm_node.cpp#L378-379) notes exactly this failure mode for open-space prompts.) So this is NOT simply "temperature too high"; do not blindly lower it.
- **Actual problems:**
  - Anyone running the node without the YAML gets meaningfully different behavior (2× rate, colder sampling, longer outputs) — silent config drift
  - No recorded experiment justifying 0.5 over 0.2/0.3; for the paper rebuttal, a JSON-parse-success-rate vs. temperature sweep would settle it
- **Severity:** MEDIUM - config drift risk; temperature choice needs empirical backing, not a blind change

---

### 8. **EGO-Planner Dynamic Constraints Never Specified** ⚠️
- **Location:** [planner_wrapper/src/ego_planner_bridge.py](../planner_wrapper/src/ego_planner_bridge.py) (bridge only; no constraint config visible)
- **Reviewer Concern:** Reviewer 3: *"The author claims that the geometric planner operates without any knowledge of the platform's dynamic. However, EGO-planner does generate a dynamically feasible trajectory requiring the kinematic limits. How are these parametrized within the EGO-planner?"*
- **Gap:** No launch file or config in this repo shows EGO-planner kinematic limits:
  - Max velocity
  - Max acceleration
  - Max yaw rate
  - These are buried in the external `ego_planner` package (not in VLNontheFly)
- **Implication:** Paper readers cannot understand what "dynamically feasible" means for this platform
- **Severity:** MEDIUM - affects evaluation and claims validation

---

## Reproducibility & Evaluation Issues

### 9. **Limited Evaluation Dataset** ⚠️
- **Issue:** Reviewer 2: *"The 15-flight evaluation is relatively limited for the reported success rate."*
- **Code/Test Evidence:**
  - [HARDWARE_TESTS.md](../ego-planner-swarm/HARDWARE_TESTS.md) shows **no automated test suite**
  - Tests are manual checklists (Stage 0–3)
  - Only 3 objects tested across 15 flights
  - Single indoor environment
- **Missing:**
  - Simulator-based ablations (reviewer suggests AirSim, Isaac Sim)
  - Baseline comparisons (e.g., Fly0, end-to-end VLA)
  - Generalization tests (different objects, lighting, viewpoints)
  - Statistical confidence intervals
- **Severity:** HIGH - affects paper validity

---

### 10. **No Ablation or Baseline Comparison** ⚠️
- **Reviewer Concern:** Reviewer 3: *"The validation of the open-vocabulary claim is currently limited... Furthermore, the protocol is not benchmarked against other methods like Fly0 or [5]."*
- **Code Reality:** No alternative implementations in repo
- **Missing Tests:**
  - Remove safety supervisor → what breaks?
  - Use coarser/finer grids
  - Compare 3×3 grid to pixel-level grounding (SAM)
  - Remove depth validation
- **Severity:** HIGH - no quantified proof of design choices

---

### 11. **Pose Robustness Not Tested** ⚠️
- **Reviewer Concern:** Reviewer 3: *"To make the paper's claims convincing, the authors should analyze the computational feasibility of running their system alongside a standard localization stack, and inject synthetic noise into the position feedback."*
- **Code Status:**
  - VIO bridge exists ([vio_bridge/src/openvins_bridge_node.cpp](../vio_bridge/src/openvins_bridge_node.cpp)) but **never tested in flights**
  - No noise injection or robustness tests
  - Only OptiTrack ground truth used
- **Severity:** MEDIUM - claims generalizability but only validated with perfect pose

---

### 12. **Power Consumption Not Reported** ⚠️
- **Reviewer Concern:** Reviewer 1: *"In addition to GPU utilization, it would be valuable to report the onboard computer's power consumption, as it directly impacts the system's autonomy."*
- **Code:** GPU profiling exists but power profiling absent
- **Impact:** Autonomous flight time unknown
- **Severity:** LOW - affects practical deployment but not algorithm correctness

---

### 13. **Inference Latency Not Explicitly Reported** ⚠️
- **Reviewer Concern:** Reviewer 2: *"The reported 37-42% GPU utilization is useful. It would also be helpful to include VLM inference latency explicitly."*
- **Code Evidence:** [edgellm_vlm_ros/src/vlm_node.cpp:259-316](../edgellm_vlm_ros/src/vlm_node.cpp#L259-316)
  - `result.ttft_ms` (time to first token) tracked
  - `result.total_ms` (total latency) tracked
  - But **not reported in paper metrics**
- **Configuration:** [vlm_node.yaml:19](../edgellm_vlm_ros/config/vlm_node.yaml#L19)
  ```yaml
  fixed_rate_hz: 0.5
  ```
  Note (corrected after recheck): 0.5 Hz is a *trigger cap*, not a latency measurement — the timer skips ticks while inference is busy ([vlm_node.cpp:561-565](../edgellm_vlm_ros/src/vlm_node.cpp#L561-565), `skip:busy`). It is *consistent with* multi-second inference but does not prove it. The node already measures `ttft_ms` and `total_ms` per result; the numbers exist, they just were not reported.
- **Severity:** MEDIUM - the data is already collected in the result JSON; reporting it is cheap

---

## Design & Implementation Gaps

### 14. **Frame Convention Complexity Not Documented** ⚠️
- **Locations:**
  - [planner_wrapper/src/ego_planner_bridge.py:17-21](../planner_wrapper/src/ego_planner_bridge.py#L17-21) (ENU→NED)
  - [ego-planner-swarm/raptor_path_tracker.py:39-41](../ego-planner-swarm/raptor_path_tracker.py#L39-41) (ENU→NED)
  - [ego-planner-swarm/raptor_path_tracker.py:151](../ego-planner-swarm/raptor_path_tracker.py#L151) (NED yaw formula)
  - [edgellm_vlm_ros/scripts/vlm_point_gate.py](../edgellm_vlm_ros/scripts/vlm_point_gate.py#L200-210) (optical→body)
- **Issue:** Multiple frame transformations (ENU, NED, optical, body, FLU, FRD) with no single diagram
- **Error Risk:** Yaw formula `yaw_ned = pi/2 - yaw_enu` easy to get wrong; see [HARDWARE_TESTS.md:68-70](../ego-planner-swarm/HARDWARE_TESTS.md#L68-70):
  ```
  > **2026-06-12, bench:** Converter orientation bug found and fixed (yaw was −90° off...)
  ```
- **Severity:** MEDIUM - already fixed but indicates fragility

---

### 15. **Depth Camera Assumptions Brittle** ⚠️
- **Location:** [edgellm_vlm_ros/config/region_gate.yaml](../edgellm_vlm_ros/config/region_gate.yaml)
- **Issue:** Multiple hardcoded depth parameters:
  ```yaml
  depth_scale_m: 0.001        # Assumes RealSense depth unit
  min_depth_m: 0.25           # Below this = invalid
  max_depth_m: 4.0            # Above this = noise
  min_clearance_m: 0.8        # Min safe distance to obstacle
  ```
- **Problem:** Code has no validation that camera actually delivers these specs
  - Different RealSense models have different ranges
  - Lighting/texture affects depth quality
  - Near-field depth unreliable
- **No Fallback:** If depth fails, region gate falls back to "most open cell by depth alone" — but if depth is broken, this is unsafe
- **Severity:** MEDIUM - robustness issue

---

### 16. **Goal Gate Configuration Manual & Error-Prone** ⚠️
- **Location:** [ego-planner-swarm/HARDWARE_TESTS.md:162-237](../ego-planner-swarm/HARDWARE_TESTS.md#L162-237)
- **Issue:** Goal gate (safety zone) must be surveyed manually and hardcoded:
  ```python
  {'fsm/goal_gate_x_min': -2.5},
  {'fsm/goal_gate_y_min': -2.5},
  {'fsm/keepout_x': [2.9, -2.9]},
  ```
- **Gap:** No YAML config file in VLNontheFly repo; parameters are in external ego_planner package
- **Risk:** If room changes or OptiTrack origin drifts, entire system fails silently
- **Severity:** LOW - detected during setup but not automated

---

### 17. **State Machine Modes Undocumented** ⚠️
- **Location:** [edgellm_vlm_ros/scripts/vlm_nav_supervisor.py:15-18](../edgellm_vlm_ros/scripts/vlm_nav_supervisor.py#L15-18)
- **Issue:** State constants defined but transitions never diagrammed:
  ```python
  POINT_NAV = "POINT_NAV"
  ALTITUDE_ADJUST = "ALTITUDE_ADJUST"
  SETTLE = "SETTLE"
  HOLD = "HOLD"
  ```
- **Missing:**
  - State diagram showing all transitions
  - Timeout/abort conditions per state
  - Recovery strategy from HOLD
- **Severity:** LOW - code is readable but docs would help

---

### 18. **Configuration Scattered Across Multiple Files** ⚠️
- **Issue:** No single source of truth for system parameters:
  - [vlm_node.yaml](../edgellm_vlm_ros/config/vlm_node.yaml) — VLM generation
  - [nav_supervisor.yaml](../edgellm_vlm_ros/config/nav_supervisor.yaml) — supervisor FSM
  - [region_gate.yaml](../edgellm_vlm_ros/config/region_gate.yaml) — depth/grid parameters
  - [point_gate.yaml](../edgellm_vlm_ros/config/point_gate.yaml) — pixel projection
  - [primitive_gate.yaml](../edgellm_vlm_ros/config/primitive_gate.yaml) — movement primitives
  - External ego_planner config for dynamics/obstacles
  - Hardcoded in [ego_raptor.launch.py](../planner_wrapper/launch/ego_raptor.launch.py) and [vlm_node.cpp](../edgellm_vlm_ros/src/vlm_node.cpp)
- **Problem:** Parameter tuning requires editing multiple files; no master config
- **Severity:** LOW - affects maintainability, not correctness

---

## Transform Frame (TF) Issues

### 19. **Deliberately TF-Free Pipeline — Consistent, but Undocumented and Roll/Pitch-Blind** ⚠️
- **Location:** [planner_wrapper/src/relative_goal_to_map.py:76-85](../planner_wrapper/src/relative_goal_to_map.py#L76-85), [planner_wrapper/src/ego_planner_bridge.py:28-31](../planner_wrapper/src/ego_planner_bridge.py#L28-31)
- **Corrected framing after recheck:** the stack avoids TF *by design*, and does so consistently: gates take pose from `/fmu/out/vehicle_local_position` directly, `relative_goal_to_map` composes goals from `/odometry`, and `ego_planner_bridge` explicitly documents "this bridge does NOT touch TF." For a PX4 companion pipeline this is a defensible, common pattern — not automatically a defect.
- **What is still genuinely problematic:**
  - `relative_goal_to_map` composes body-relative goals using **yaw only**:
    ```python
    out.pose.position.x = float(pos.x + cy * rel.x - sy * rel.y)
    out.pose.position.y = float(pos.y + sy * rel.x + cy * rel.y)
    ```
    Roll/pitch are ignored. Fine near hover; wrong if the goal is composed while the drone is pitched during motion (a forward goal computed at 10° pitch lands short/long). The gates also freeze the pose at execute time, not at image-capture time, adding pose/image skew at ~1 s VLM latency.
  - Nothing validates that the three manual frame chains (optitrack_bridge NED, odometry_converter ENU, gate NED→ENU) stay consistent — a yaw-convention bug of exactly this kind already happened and was caught only on the bench ([HARDWARE_TESTS.md:68-70](../ego-planner-swarm/HARDWARE_TESTS.md#L68-70): "yaw was −90° off in both odometry converters")
  - The TF-free design decision is not written down anywhere; a new contributor will "fix" it by adding TF and break assumptions
- **Severity:** MEDIUM (downgraded from HIGH after recheck) - the design is coherent; the risks are the undocumented convention and the yaw-only approximation

---

### 20. **OptiTrack TF Broadcast Exists but Is Disabled in the Flight Launch** ℹ️
- **Location:** [vio_bridge/src/optitrack_bridge_node.cpp:50](../vio_bridge/src/optitrack_bridge_node.cpp#L50), [planner_wrapper/launch/ego_raptor.launch.py:202-203](../planner_wrapper/launch/ego_raptor.launch.py#L202-203)
- **Correction:** an earlier version of this report claimed the bridge "doesn't publish TF." That was **wrong**: the node's own default is `publish_tf = true` and it broadcasts `odom_ned → base_link_frd`. The flight launch overrides it to `false` ("Keep false for ego-planner").
- **Remaining (smaller) issue:** the frames it broadcasts are NED/FRD (`odom_ned`, `base_link_frd`), while the planner/RViz world is ENU `map` with FLU `base_link`. So even when enabled, this TF does not provide the `map → base_link` transform ROS tools expect — it's a parallel NED tree. The launch comment says to keep it off but not why (probably to avoid a confusing second tree). One explanatory sentence in the launch file would close this.
- **Severity:** LOW (downgraded) - cosmetic/documentation issue, not a functional gap

---

### 21. **Camera Mount TF Is Published but Never Consumed by the Goal Math** ⚠️
- **Location:** [planner_wrapper/launch/ego_raptor.launch.py:99-117](../planner_wrapper/launch/ego_raptor.launch.py#L99-117) (mount TF: x=-0.155 m, pitch=-7°) vs. [edgellm_vlm_ros/scripts/vlm_point_gate.py:268-272](../edgellm_vlm_ros/scripts/vlm_point_gate.py#L268-272)
- **Sharpened finding after recheck:** the launch publishes a `base_link → camera_link` mount TF with a **-7° pitch and a -0.155 m translation**, but the gates' deprojection ignores it entirely:
  ```python
  def optical_to_body_horizontal(point_optical):
      optical_x, _optical_y, optical_z = point_optical
      body_x = optical_z      # assumes camera optical axis == body forward
      body_y = -optical_x
      return body_x, body_y
  ```
  The published mount TF is effectively decorative for goal projection — it exists for RViz, not for the math.
- **Quantified impact:**
  - **Lever arm:** every goal is offset by the 0.155 m camera-to-base_link translation — a constant ~15 cm bias, the same order as the reviewer-questioned 20 cm success radius
  - **Pitch:** at -7°, a point at image center and depth *d* has true forward component *d·cos 7°* (-0.75%, negligible) but the vertical offset *d·sin 7°* ≈ 0.12·*d* silently leaks into the forward estimate for off-center pixels; mostly masked because the gate discards vertical and holds altitude
- **Why it hasn't visibly failed:** `standoff_m: 0.8` and `max_goal_distance_m: 1.5` with continuous replanning absorb a 15 cm bias. But it directly interacts with close-range accuracy — the exact regime Reviewer 3 asked about.
- **Fix:** apply the mount extrinsics (at minimum the translation and pitch) in `optical_to_body_horizontal`, sourcing the same values the launch file uses so they can't diverge.
- **Severity:** MEDIUM - systematic ~15 cm goal bias, currently masked by standoff margins

---

### 22. **Startup Input Validation: Mostly Present (correction)** ℹ️
- **Correction after recheck:** an earlier version of this report claimed the gates lack startup TF validation. Since the gates don't use TF at all (see #19), that check would be meaningless. What the gates *actually* do is validate their real inputs before every goal publication ([vlm_region_gate.py:468-490](../edgellm_vlm_ros/scripts/vlm_region_gate.py#L468-490)): no depth image → reject, no camera info → reject, stale result/depth/pose (`max_result_age_s`/`max_depth_age_s`/`max_pose_age_s`) → reject. If RealSense dies, goals stop within 0.5 s (`max_depth_age_s`). This is solid.
- **Remaining small gap:** the depth image and the pose are checked for *age* but not for *mutual consistency* — the pose used for goal composition can be up to 1 s newer than the depth frame the goal was derived from, and the VLM result can be based on an RGB frame up to 2.5 s older than execution. At 0.5 m/s that's up to ~1.2 m of drone motion between "what the VLM saw" and "where the goal is executed from."
- **Severity:** LOW - the staleness windows bound the error, but the capture-time-vs-execute-time skew is worth stating in the paper

---

## Code Quality Issues

### 23. **Limited Test Coverage** ⚠️
- **Location:** [edgellm_vlm_ros/test/](../edgellm_vlm_ros/test/)
- **Issue:**
  - Unit tests exist for gate logic (vlm_point_gate, vlm_region_gate, vlm_primitive_gate, vlm_nav_supervisor)
  - But **no integration tests** combining gates + supervisor
  - No end-to-end simulation tests
  - No failure injection tests (e.g., VLM timeout, bad depth)
- **Severity:** LOW - test infrastructure exists but incomplete

---

### 24. **JSON Parsing Fragility** ⚠️
- **Location:** [edgellm_vlm_ros/scripts/vlm_region_gate.py:118-153](../edgellm_vlm_ros/scripts/vlm_region_gate.py#L118-153)
- **Code:** Fallback parsing with regex and keyword matching:
  ```python
  def _match_region_keyword(text: str, known: List[str]) -> Optional[str]:
      norm = re.sub(r"[^A-Z-]", "", str(text).upper().replace("_", "-").replace(" ", "-"))
      for name in sorted(known, key=len, reverse=True):
          if name in norm:
              return name
  ```
- **Issue:** Masks underlying VLM output quality issues
  - If VLM gets confused, regex recovery might guess wrong
  - No confidence signal returned from fallback parsing
- **Severity:** LOW - defensive but not ideal

---

## Documentation Gaps

### 25. **No End-to-End Architecture Diagram** ⚠️
- **Gap:** Paper and README lack a single diagram showing:
  - Data flow (camera → VLM → region gate → supervisor → planner → Raptor → PX4)
  - Frame conventions (ENU, NED, optical, body, map)
  - Failsafe logic (MoCAP → EKF, timeout handling, RC override)
- **Current State:** Scattered across multiple launch files and node code
- **Severity:** LOW - nice-to-have for understanding

---

### 26. **Qwen Model Version & Licensing Unclear** ⚠️
- **Code Evidence:** [vlm_node.yaml:7-8](../edgellm_vlm_ros/config/vlm_node.yaml#L7-8)
  ```yaml
  engine_dir: ".../Qwen3.5-2B/int4_awq/..."
  ```
- **Issue:** Model version (3.5-2B) but:
  - No mention of fine-tuning (if any)
  - No license/attribution (Qwen is Apache 2.0, but TensorRT build has different license)
  - No weights/config in repo (non-reproducible)
- **Severity:** MEDIUM - affects reproducibility and licensing compliance

---

### 27. **Dead Ternary in Image Conversion** ℹ️ (found during recheck)
- **Location:** [edgellm_vlm_ros/src/vlm_node.cpp:505](../edgellm_vlm_ros/src/vlm_node.cpp#L505)
- **Code:**
  ```cpp
  frame.rgb = rgb.isContinuous() ? rgb.clone() : rgb.clone();
  ```
- **Issue:** Both branches are identical — the `isContinuous()` check is dead. The intent was presumably to skip the clone for already-continuous mats (`rgb.isContinuous() ? rgb : rgb.clone()`), but note the RGB8 path aliases the cv_bridge buffer, so an unconditional `rgb.clone()` is actually the *safe* choice there. The fix is to delete the ternary, not to "restore" it.
- **Severity:** LOW - harmless, but a tell-tale of an unfinished edit

---

## Summary Table

| # | Category | Issue | Severity | Location |
|---|----------|-------|----------|----------|
| 1 | Architecture | MoCAP dependency vs. "fully onboard" claim | HIGH | ego_raptor.launch.py |
| 2 | Documentation | ESDF unexplained | MEDIUM | Paper (not code) |
| 3 | Design | 3×3 grid: no hysteresis/edge-clip detection (standoff/cooldown mitigations exist but undocumented) | MEDIUM | vlm_region_gate.py |
| 4 | Documentation | Safety logic spread over gates + supervisor + goal gate, never enumerated | MEDIUM | vlm_nav_supervisor.py + gates |
| 5 | Reproducibility | Hardcoded `/home/orin/…` paths | HIGH | vlm_node.yaml, .cpp |
| 6 | Documentation | VLM prompting mechanism (grid is prompt-text, not overlay) undocumented | MEDIUM | vlm_node.cpp |
| 7 | Reproducibility | C++ defaults diverge from YAML config (temp 0.2 vs 0.5, rate 1.0 vs 0.5) | MEDIUM | vlm_node.cpp vs vlm_node.yaml |
| 8 | Documentation | EGO-planner kinematic limits unspecified in paper | MEDIUM | external ego_planner pkg |
| 9-10 | Evaluation | 15 flights, 3 objects, no ablations/baselines | HIGH | paper |
| 11 | Robustness | Pose robustness untested; VIO bridge exists but unflown | MEDIUM | vio_bridge |
| 12 | Evaluation | Power consumption unreported | LOW | — |
| 13 | Evaluation | ttft/total latency measured per-result but unreported | MEDIUM | vlm_node.cpp |
| 14 | Architecture | Frame-convention complexity (a yaw bug already occurred once) | MEDIUM | converters/bridges |
| 15 | Robustness | Depth parameter assumptions unvalidated | MEDIUM | region_gate.yaml |
| 16 | Ops | Goal gate surveyed and hardcoded manually | LOW | external launch |
| 17 | Documentation | Supervisor FSM not diagrammed | LOW | vlm_nav_supervisor.py |
| 18 | Maintainability | Config spread over 5+ files | LOW | config/*.yaml |
| 19 | Architecture | TF-free-by-design pipeline: undocumented, yaw-only goal composition | MEDIUM | relative_goal_to_map.py |
| 20 | Documentation | OptiTrack NED TF disabled in launch without stating why (node default is on) | LOW | ego_raptor.launch.py |
| 21 | Correctness | Camera mount TF (-7° pitch, -0.155 m) published but ignored by goal math → ~15 cm systematic bias | MEDIUM | vlm_point_gate.py |
| 22 | Robustness | Image-capture vs. goal-execution time skew (up to ~2.5 s / ~1.2 m) | LOW | gates |
| 23 | Testing | No integration/failure-injection tests | LOW | test/ |
| 24 | Robustness | Fallback JSON parsing masks VLM output quality | LOW | vlm_region_gate.py |
| 25 | Documentation | No end-to-end architecture diagram | LOW | — |
| 26 | Reproducibility | Model provenance/licensing not stated | MEDIUM | vlm_node.yaml |
| 27 | Code Quality | Dead ternary (`? rgb.clone() : rgb.clone()`) | LOW | vlm_node.cpp:505 |

---

## Recommendations

### High Priority
1. **Remove `/home/orin/` hardcoding** — use ROS package paths or environment variables
2. **Quantify grid search limitations** — add tests for oscillation/edge-clipping behavior; state openly in the rebuttal that `standoff_m: 0.8` is the close-range stop mechanism
3. **Validate MoCAP claim** — either:
   - Test with VIO in flight, or
   - Rewrite paper claim to "OptiTrack-aided navigation"
4. **Document EGO-planner dynamics** — clarify kinematic limits and how they're set

### Medium Priority
5. **Compensate camera mount extrinsics in the gates** — the -7° pitch and -0.155 m lever arm are currently ignored (~15 cm goal bias)
6. **Add power/latency metrics** — `ttft_ms`/`total_ms` are already in every result JSON; aggregate and report them
7. **Test VIO robustness** — synthetic noise injection in simulation
8. **Document grid overlay mechanism** — clarify point/region/primitive VLM interfaces (grid lives in the prompt text, not on the image)
9. **Reconcile C++ defaults with YAML config** — or make the params mandatory (no default) so drift is impossible; back the temperature choice with a parse-success sweep

### Low Priority
9. **Add state machine diagram** — clarify supervisor FSM
10. **Centralize configuration** — single master config file with all parameters
11. **Extend test coverage** — integration tests for gate + supervisor chains

---

## Files Requiring Updates

- [ ] [vlm_node.yaml](../edgellm_vlm_ros/config/vlm_node.yaml) — remove hardcoded paths
- [ ] [vlm_node.cpp](../edgellm_vlm_ros/src/vlm_node.cpp) — dynamic path loading
- [ ] [ego_raptor.launch.py](../planner_wrapper/launch/ego_raptor.launch.py) — document MoCAP dependency
- [ ] [vlm_region_gate.py](../edgellm_vlm_ros/scripts/vlm_region_gate.py) — add oscillation/edge-clip tests
- [ ] [vlm_nav_supervisor.py](../edgellm_vlm_ros/scripts/vlm_nav_supervisor.py) — clarify role vs. true safety layer
- [ ] [HARDWARE_TESTS.md](../ego-planner-swarm/HARDWARE_TESTS.md) — document dynamic constraints
- [ ] Project README — add architecture diagram and frame conventions

---

**End of Report**
