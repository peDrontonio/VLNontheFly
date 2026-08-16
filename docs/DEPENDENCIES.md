# Dependencies: what is fetched, what is vendored, what you must download

Nothing in this repository is a mystery blob. Every third-party component is
either fetched at a pinned version, or vendored with its divergence from
upstream written down. Every model weight is listed with its source.

---

## 1. Fetched sources

```bash
vcs import < third_party.repos
```

Declared in [`third_party.repos`](../third_party.repos):

| Path | Upstream | Pin |
|---|---|---|
| `px4_msgs/` | [PX4/px4_msgs](https://github.com/PX4/px4_msgs) | `392e831c` (**PX4 v1.16.2**) |
| `planner_wrapper/imports/navdp/depth_anything/` | [DepthAnything/Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2) | `a561b849` |

Both pins were **verified, not assumed**. Before these packages were removed
from the tree, the git tree hash of the vendored copy was compared against
every upstream branch and tag; the pins above are the commits that match
byte-for-byte, so what you fetch is what was flown.

- `px4_msgs/msg` = `ff5176c2d2dbb335460c97eaf0c9d6a1f6c2afc4` → matches only `v1.16.2`
- `depth_anything/depth_anything_v2` = `01a34f1ddd91ab3fdfaac67833d1bb53ea3cf10d` → matches `a561b849`

The old `package.xml` claimed px4_msgs "2.0.1", which corresponds to no
lookup-able PX4 release. That is why the pin is a commit and the PX4 version is
stated next to it.

---

## 2. Vendored sources, and why

### `realsense-ros/` — a modified fork, deliberately kept in-tree

This is **not** stock realsense-ros, and it must not be replaced by an upstream
fetch. Against upstream tag `4.57.7` it carries two local changes, recorded in
[`../patches/realsense2_camera-vln-fork.patch`](../patches/realsense2_camera-vln-fork.patch):

1. **Camera-mount static TF.** The D435i is bolted to the airframe and has no
   URDF, so nothing published `base_link` → `camera_link`. Goal projection needs
   that transform to turn a VLM-grounded image region into a world-frame point.
   Adds these parameters to `realsense2_camera`:

   | Parameter | Type | Default |
   |---|---|---|
   | `publish_mount_tf` | bool | `false` |
   | `mount_parent_frame_id` | string | `base_link` |
   | `mount_x`, `mount_y`, `mount_z` | double | `0.0` (metres) |
   | `mount_roll`, `mount_pitch`, `mount_yaw` | double | `0.0` (radians) |

   With `publish_mount_tf:=false` the driver behaves exactly like upstream.

2. **librealsense SDK pin** lowered `2.57.7` → `2.56.6` in `CMakeLists.txt`, to
   match the `librealsense2` build on the Jetson Orin NX image.

Only `realsense2_camera` and its `realsense2_camera_msgs` dependency are kept.
The description meshes, the RGBD plugin and the MQTT bridge were removed —
nothing in this workspace referenced them.

> If you ever rebase this fork onto a newer realsense-ros, re-generate the patch
> file so the record stays true.

### `ego-planner-swarm/` — a modified fork

Carries `raptor_path_tracker.py` and goal-gate changes on top of upstream
EGO-Planner. The upstream simulator and swarm packages were removed; see the
"drop simulation-only" commit for the reasoning.

---

## 3. Model weights

**No weights are in git, and none ever can be** — `*.pt`, `*.ckpt`, `*.pth`,
`*.engine` and `*.onnx` are gitignored, and they are far past GitHub's file
limits. This section is the download list.

| Weight | Needed by | Status | Where it comes from |
|---|---|---|---|
| Qwen-3.5-2B, INT4 TensorRT engine | `edgellm_vlm_ros` | **required for the validated flow** | Built locally from a TensorRT-Edge-LLM checkout — see [`INSTALL.md`](INSTALL.md). Engines are device- and TensorRT-version-specific, so this is a build, not a download. |
| `depth-anything/Depth-Anything-V2-Small-hf` | `depth_estimator` | experimental | Downloaded automatically by `transformers` on first run. No manual step. Override with the node's `model_name` parameter. |
| `navdp.ckpt` (543 MB) | `planner_wrapper` NavDP | **experimental, never obtained** | Upstream NavDP release. What is currently on the flight machine is an unresolved Git LFS pointer, `sha256:3bb3ad4ab241e857bb57a4021cc6aab76d5263e81fbf80298d579053ef011947`. |
| `iplanner.pt` (213 MB) | `planner_wrapper` iPlanner | **experimental, never obtained** | Upstream iPlanner release. Also an unresolved LFS pointer, `sha256:685f16cde28d05249d50d24ed79ab4bdc94b3fbbcb99c8dbaed31039d11633b9`. |
| RAPTOR policy | flight controller | n/a | Lives in the PX4/rl-tools firmware on the Pixhawk, not in this repo. See [`EXTERNAL_MODE_STATUS.md`](EXTERNAL_MODE_STATUS.md). |

### About NavDP and iPlanner

Those two 134-byte files under `planner_wrapper/checkpoints/` are Git LFS
pointers that were never resolved, which means **the NavDP and iPlanner nodes
have never run in this workspace**. They are marked experimental for that
reason. If you obtain the weights, drop them at:

```
planner_wrapper/checkpoints/navdp.ckpt
planner_wrapper/checkpoints/iplanner.pt
```

and pass the path via each node's `checkpoint` parameter. The build skips the
directory when it is absent, so you do not need them for anything else.
