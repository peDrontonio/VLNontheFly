# Transform Frame (TF) Issues — Addendum (Rechecked)
**Revised 2026-07-13 after re-verification against the code. The first version of this
addendum contained two factual errors; they are corrected and marked below.**

---

## Executive Summary

The stack is **deliberately TF-free**: gates read pose from `/fmu/out/vehicle_local_position`,
goal composition uses `/odometry`, and `ego_planner_bridge.py` explicitly documents "this
bridge does NOT touch TF." That is a coherent, common pattern for PX4 companion pipelines —
the earlier version of this report treated it as a defect, which was too harsh.

After rechecking, the real TF-related findings are:

| # | Finding | Severity | Status after recheck |
|---|---------|----------|----------------------|
| 1 | Camera mount TF (-7° pitch, -0.155 m) published but **never used** by the goal math | **MEDIUM** | Confirmed, sharpened |
| 2 | Yaw-only goal composition ignores roll/pitch; frame chains unvalidated | MEDIUM | Confirmed, reframed |
| 3 | OptiTrack TF disabled in launch without explanation | LOW | **Corrected** — node publishes TF by default |
| 4 | "No TF validation at startup" | — | **Retracted** — gates don't use TF; their actual input validation is solid |

---

## Finding 1 (MEDIUM): Camera Mount Extrinsics Are Published but Ignored

The strongest genuine TF finding, missed in the first pass.

**The launch file** ([ego_raptor.launch.py:99-117](../planner_wrapper/launch/ego_raptor.launch.py#L99-117))
publishes a `base_link → camera_link` mount TF with real extrinsics:
- translation `x = -0.155 m`
- pitch `-7°`

**The gates never consume it.** [vlm_point_gate.py:268-272](../edgellm_vlm_ros/scripts/vlm_point_gate.py#L268-272)
hardcodes an identity camera-to-body alignment:

```python
def optical_to_body_horizontal(point_optical):
    optical_x, _optical_y, optical_z = point_optical
    body_x = optical_z      # assumes optical axis == body forward
    body_y = -optical_x
    return body_x, body_y
```

`vlm_region_gate.py` reuses the same function. The published mount TF only serves RViz.

**Impact:**
- Constant **~15 cm forward bias** on every projected goal (the lever arm) — the same order
  of magnitude as the 20 cm success radius the reviewer questioned
- The -7° pitch contributes <1% depth error on-axis; it is mostly masked because the gate
  discards the vertical component and holds altitude (`goal_z_mode: current_pose`)
- In practice absorbed by `standoff_m: 0.8` + `max_goal_distance_m: 1.5` + continuous
  replanning — which is why it hasn't visibly failed — but it sits exactly in the
  close-range-accuracy regime Reviewer 3 asked about

**Fix:** apply the mount translation (and pitch, if precision matters) inside
`optical_to_body_horizontal`, sourced from the same parameters the launch file uses so the
two cannot drift apart.

---

## Finding 2 (MEDIUM): Yaw-Only Goal Composition, Unvalidated Frame Chains

[relative_goal_to_map.py:76-85](../planner_wrapper/src/relative_goal_to_map.py#L76-85)
composes body-relative goals into map goals using yaw only:

```python
yaw = yaw_from_quaternion(odom.pose.pose.orientation)
out.pose.position.x = float(pos.x + cy * rel.x - sy * rel.y)
out.pose.position.y = float(pos.y + sy * rel.x + cy * rel.y)
```

**What's fine about this:** near hover, a multirotor's roll/pitch are small, and yaw-only
composition is standard practice in PX4 pipelines. Recommending "just use TF2" (as the first
version of this addendum did) would fight the stack's consistent odometry-based design for
little gain.

**What's genuinely risky:**
1. **Roll/pitch are not small in motion.** A goal composed while the drone is pitched ~10°
   mid-flight places the "forward" offset short/long. The gates also pair the *latest* pose
   with a VLM result derived from an image up to 2.5 s old (`max_result_age_s`), so at
   0.5 m/s the drone may have moved ~1.2 m between what the VLM saw and where the offset is
   applied.
2. **Three hand-written frame chains, zero cross-checks.** optitrack_bridge (ENU→NED),
   odometry_converter (NED→ENU), gates (NED→ENU via `px4_local_position_to_pose`). A yaw
   convention bug of exactly this class already happened and was caught only on the bench:
   [HARDWARE_TESTS.md:68-70](../ego-planner-swarm/HARDWARE_TESTS.md#L68-70) — *"yaw was
   −90° off in both odometry converters."*
3. **The TF-free convention is undocumented.** It lives in scattered comments; nothing stops
   a contributor from introducing TF in one node and silently breaking assumptions.

**Fix:** a short FRAMES.md stating the convention (which topics are ENU/NED, who converts
where, why TF is not used), plus a bench check-script that cross-validates the three chains
against each other (drive one input, assert all outputs agree).

---

## Finding 3 (LOW, corrected): OptiTrack TF Broadcast Exists — the Launch Turns It Off

**Correction:** the first version of this addendum claimed "OptiTrack Bridge Doesn't Publish
TF." That was **wrong**. [optitrack_bridge_node.cpp:50](../vio_bridge/src/optitrack_bridge_node.cpp#L50)
defaults `publish_tf = true` and broadcasts `odom_ned → base_link_frd`
([line 198-200](../vio_bridge/src/optitrack_bridge_node.cpp#L198-200)). The flight launch
overrides it to `false` with the comment "Keep false for ego-planner."

**What remains worth noting:**
- Even when enabled, the broadcast frames are NED/FRD (`odom_ned`, `base_link_frd`) — a
  parallel tree, not the ENU `map → base_link` that RViz/ROS tools want alongside the
  planner. Enabling it does not give you standard visualization; it may be disabled
  precisely to avoid a confusing second tree.
- The launch comment states *what* to do but not *why*. One sentence would prevent someone
  from flipping it on expecting `map → base_link`.

**Do not** blindly change the default to `true` (the first version of this addendum
suggested that; retracted).

---

## Finding 4 (retracted): "No TF Validation at Startup"

Retracted as meaningless: the gates don't use TF, so there is no TF tree to validate. Their
actual input validation is good — every goal publication checks for presence and freshness
of depth image, camera info, pose, and VLM result
([vlm_region_gate.py:468-490](../edgellm_vlm_ros/scripts/vlm_region_gate.py#L468-490));
if RealSense dies, goals stop within `max_depth_age_s = 0.5 s`.

The residual (LOW) observation is the **capture-vs-execute time skew** described under
Finding 2: ages are checked individually, but the pose/image pairing is not synchronized.

---

## Testing Checklist (revised)

- [ ] Measure the ~15 cm lever-arm bias: place a target at a surveyed position, run the
      region gate, compare published goal vs. ground truth (OptiTrack makes this easy)
- [ ] Compose a goal while the drone is hand-tilted ~10–15° and confirm the offset error
      matches the yaw-only prediction
- [ ] Frame cross-check script: feed one known pose through optitrack_bridge,
      odometry_converter, and `px4_local_position_to_pose`; assert consistency
      (automates the bench check that caught the −90° yaw bug)
- [ ] `ros2 run tf2_tools view_frames` with `publish_tf:=true` to confirm the NED tree is
      what you expect (`odom_ned → base_link_frd`), and document it

---

## Summary

| Finding | Severity | One-line takeaway |
|---------|----------|-------------------|
| Mount extrinsics ignored by goal math | MEDIUM | ~15 cm systematic goal bias, masked by standoff margins |
| Yaw-only composition + unvalidated chains | MEDIUM | Fine at hover; add FRAMES.md + a cross-check script |
| OptiTrack TF off in launch | LOW | Node publishes NED TF by default; launch disables it — document why |
| TF validation at startup | retracted | Gates are TF-free; their input validation is already solid |
