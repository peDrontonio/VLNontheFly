# AI Slop Detections Report
**VLN on the Fly — Codebase Analysis Against IMAV 2026 Reviewer Feedback**

**Report generated:** 2026-07-13 · **Rechecked and corrected:** 2026-07-13 (same day, second pass)

---

## What's Here

An audit of the VLNontheFly codebase against the IMAV 2026 reviewer feedback
([review/imav2026_reviewer_feedback.md](../review/imav2026_reviewer_feedback.md)), mapping
each reviewer concern to code locations, plus additional issues found independently.

1. **[CODE_REVIEW_FINDINGS.md](CODE_REVIEW_FINDINGS.md)** — 27 findings with file/line references,
   severity ratings, and a summary table
2. **[TF_ISSUES_ADDENDUM.md](TF_ISSUES_ADDENDUM.md)** — transform/frame analysis (rewritten
   after recheck; see corrections below)
3. **[ACTIONABLE_FIXES.md](ACTIONABLE_FIXES.md)** — implementation steps with code examples

---

## ⚠️ Corrections Made During Recheck

The first pass contained errors of its own. A full re-verification against the code found
and fixed the following — read the current documents, not cached impressions of the first pass:

| First-pass claim | What the code actually shows |
|---|---|
| "No mitigation in code" for grid oscillation/edge-clipping | Partial mitigations exist: `cooldown_s: 2.0`, `standoff_m: 0.8`, `max_goal_distance_m: 1.5`, `min_confidence`, median-over-cell depth. Missing: explicit hysteresis and edge-clip detection. Severity downgraded HIGH→MEDIUM. |
| "Safety supervisor has no actual safety validation" | Physical validation lives in the **gates** (depth, clearance, staleness, clamps) and the planner-side **goal gate** (box + keep-outs). The supervisor is only the mode FSM. Real issue: the paper never enumerates the distributed safety rules. |
| "OptiTrack bridge doesn't publish TF" | **Wrong.** The node defaults `publish_tf=true` and broadcasts `odom_ned → base_link_frd`; the flight launch disables it deliberately. |
| "Temperature 0.5 too high, lower to 0.2" | The 0.5 is intentional and documented in the YAML (too-cold sampling collapses the small VLM to always-CENTER). Real issue: C++ defaults (0.2, 1.0 Hz) diverge from YAML (0.5, 0.5 Hz), and the choice lacks recorded evidence. |
| "fixed_rate_hz 0.5 proves latency > 2 s" | Overstated — 0.5 Hz is a trigger cap; ticks skip while busy. Latency (`ttft_ms`, `total_ms`) is already measured per-result, just unreported. |
| "No TF validation at startup" (as a gate defect) | Retracted — the gates are TF-free by design and their input freshness validation is solid. Residual: image-capture vs. goal-execution time skew (up to ~2.5 s). |
| Profiler sample code (power via thermal_zone grep) | Was nonsense; replaced with INA3221 hwmon / tegrastats approach. |
| Issue-count arithmetic ("30 issues, was 22 plus 4") | Wrong math. Correct current count: **27 findings** (see below). |

**New findings from the recheck:**
- **Camera mount extrinsics ignored by goal math (MEDIUM):** the launch publishes a
  `base_link → camera_link` TF with -7° pitch and -0.155 m translation, but
  `optical_to_body_horizontal` assumes identity alignment → ~15 cm systematic goal bias,
  currently masked by standoff margins. This is the strongest TF finding.
- **Dead ternary** at vlm_node.cpp:505: `rgb.isContinuous() ? rgb.clone() : rgb.clone()`.
- The 0.8 m standoff is the honest answer to Reviewer 3's "was the 20 cm radius chosen to
  stop before grid issues manifest" — the pipeline stops 0.8 m short **by design**; say so
  in the rebuttal.

---

## Top Issues (post-recheck)

| # | Issue | Severity | Where |
|---|-------|----------|-------|
| 1 | Hardcoded `/home/orin/…` paths block reproducibility | HIGH | vlm_node.yaml / .cpp / launch |
| 2 | "Fully onboard" claim vs. OptiTrack dependency (VIO bridge exists, unflown) | HIGH | ego_raptor.launch.py, vio_bridge |
| 3 | 15 flights / 3 objects / 1 environment, no ablations or baselines | HIGH | paper + HARDWARE_TESTS.md |
| 4 | Camera mount extrinsics published but ignored → ~15 cm goal bias | MEDIUM | vlm_point_gate.py:268 |
| 5 | Grid search lacks hysteresis / edge-clip detection (partial mitigations exist) | MEDIUM | vlm_region_gate.py |
| 6 | Safety rules distributed across gates+supervisor+goal-gate, never enumerated | MEDIUM | multiple |

## Severity Breakdown (27 findings)

- **HIGH (4):** hardcoded paths; MoCAP vs. paper claim; limited evaluation; no ablations/baselines
- **MEDIUM (13):** ESDF unexplained; grid hysteresis/edge-clip; safety-rule documentation;
  prompting mechanism; C++/YAML config drift; EGO-planner dynamics; pose robustness untested;
  latency unreported; frame-convention fragility; depth assumptions; TF-free design undocumented
  (yaw-only composition); mount extrinsics ignored; model provenance
- **LOW (10):** power unreported; goal-gate manual survey; FSM undiagrammed; scattered config;
  OptiTrack TF launch comment; capture/execute time skew; test coverage; JSON-parse fallback;
  no architecture diagram; dead ternary

---

## How to Use

- **Rebuttal writing:** start from the Summary Table in
  [CODE_REVIEW_FINDINGS.md](CODE_REVIEW_FINDINGS.md); items 3, 4, 13, 21 give concrete,
  honest answers to specific reviewer questions.
- **Code fixes:** [ACTIONABLE_FIXES.md](ACTIONABLE_FIXES.md), in priority order. Fix 1
  (paths) and the mount-extrinsics compensation give the most value per effort.
- **Frames/TF:** [TF_ISSUES_ADDENDUM.md](TF_ISSUES_ADDENDUM.md) — includes a bench checklist
  to measure the 15 cm bias and cross-validate the three hand-written frame chains.

---

## Files Referenced

- [review/imav2026_reviewer_feedback.md](../review/imav2026_reviewer_feedback.md)
- [edgellm_vlm_ros/config/vlm_node.yaml](../edgellm_vlm_ros/config/vlm_node.yaml) · [config/region_gate.yaml](../edgellm_vlm_ros/config/region_gate.yaml) · [config/nav_supervisor.yaml](../edgellm_vlm_ros/config/nav_supervisor.yaml)
- [edgellm_vlm_ros/src/vlm_node.cpp](../edgellm_vlm_ros/src/vlm_node.cpp)
- [edgellm_vlm_ros/scripts/vlm_point_gate.py](../edgellm_vlm_ros/scripts/vlm_point_gate.py) · [scripts/vlm_region_gate.py](../edgellm_vlm_ros/scripts/vlm_region_gate.py) · [scripts/vlm_nav_supervisor.py](../edgellm_vlm_ros/scripts/vlm_nav_supervisor.py)
- [planner_wrapper/launch/ego_raptor.launch.py](../planner_wrapper/launch/ego_raptor.launch.py) · [src/relative_goal_to_map.py](../planner_wrapper/src/relative_goal_to_map.py) · [src/ego_planner_bridge.py](../planner_wrapper/src/ego_planner_bridge.py)
- [vio_bridge/src/optitrack_bridge_node.cpp](../vio_bridge/src/optitrack_bridge_node.cpp)
- [ego-planner-swarm/HARDWARE_TESTS.md](../ego-planner-swarm/HARDWARE_TESTS.md)
