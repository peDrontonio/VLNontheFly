# tests/

Every unit test in the workspace, in one tree, mirroring the package it covers.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # needs px4_msgs built, see docs/INSTALL.md
pytest tests/
```

No hardware, no drone, no model weights, no GPU. The whole suite is pure
Python and runs in about a second on the Jetson.

## Layout

| Path | Covers | Status of the code under test |
|---|---|---|
| `tests/edgellm_vlm_ros/` | the VLM goal gates and the nav supervisor in `edgellm_vlm_ros/scripts/` | **FLIGHT-VALIDATED** |

`tests/imav_bt/` arrives with the behaviour-tree package, which is a separate,
not-yet-flight-validated change.

## Why here and not in each package

`colcon test` discovers tests inside packages, which spread them across the
workspace and made "run the tests" a different command per package. These are
plain `unittest` files with no ROS node under test, so nothing needed colcon to
run them. Collecting them here gives one command and one place to look.

Tests locate the code they cover relative to the repository root, e.g.

```python
SCRIPTS = Path(__file__).resolve().parents[2] / "edgellm_vlm_ros" / "scripts"
```

so a new package's tests go in `tests/<package_name>/` and follow the same
pattern.

## Requirements

ROS 2 Humble must be sourced: the gates exchange `geometry_msgs`,
`sensor_msgs` and `px4_msgs` types, so those message packages have to be
importable. `px4_msgs` is fetched rather than vendored, so
`vcs import < third_party.repos` and a `colcon build` must have run at least
once. Everything else is stdlib plus `numpy`.

## Beyond unit tests

Unit tests are the first rung of a ladder that ends with a real flight. The
rungs above this one, and their pass criteria, are in
[`docs/TESTING.md`](../docs/TESTING.md).
