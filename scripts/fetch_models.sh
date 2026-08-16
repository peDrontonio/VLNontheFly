#!/usr/bin/env bash
#
# Check and stage the model weights this workspace can use.
#
#   ./scripts/fetch_models.sh
#
# Nothing here is needed for the FLIGHT-VALIDATED pipeline: the validated flow
# (ego_raptor.launch.py + edgellm_vlm_ros in region mode) uses the RealSense
# stereo depth and a TensorRT engine you build yourself, so a clean clone flies
# with no downloads. This script covers the EXPERIMENTAL planners.
#
# See docs/DEPENDENCIES.md for the full table.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CKPT_DIR="$REPO_ROOT/planner_wrapper/checkpoints"

# name : sha256 : bytes : what needs it
MODELS=(
  "navdp.ckpt:3bb3ad4ab241e857bb57a4021cc6aab76d5263e81fbf80298d579053ef011947:543257151:NavDP  (planner_wrapper/src/navdp.py)"
  "iplanner.pt:685f16cde28d05249d50d24ed79ab4bdc94b3fbbcb99c8dbaed31039d11633b9:213343275:iPlanner (planner_wrapper/src/iplanner.py)"
)

ok=0
missing=0

echo "Model check -- $CKPT_DIR"
echo

mkdir -p "$CKPT_DIR"

for entry in "${MODELS[@]}"; do
  name="${entry%%:*}"; rest="${entry#*:}"
  want="${rest%%:*}";  rest="${rest#*:}"
  size="${rest%%:*}"
  used_by="${rest#*:}"
  path="$CKPT_DIR/$name"

  printf '  %-14s ' "$name"

  if [[ ! -f "$path" ]]; then
    echo "MISSING        needed by $used_by"
    missing=$((missing + 1))
    continue
  fi

  # A Git LFS pointer is a ~134-byte text file, not the weights. This is the
  # state the flight machine has been in: the LFS objects were never pulled.
  if head -c 42 "$path" 2>/dev/null | grep -q 'git-lfs.github.com/spec'; then
    echo "LFS POINTER    not the real weights, needed by $used_by"
    missing=$((missing + 1))
    continue
  fi

  have="$(sha256sum "$path" | cut -d' ' -f1)"
  if [[ "$have" == "$want" ]]; then
    echo "OK             $(numfmt --to=iec "$size")"
    ok=$((ok + 1))
  else
    echo "CHECKSUM MISMATCH"
    echo "                   expected $want"
    echo "                   got      $have"
    missing=$((missing + 1))
  fi
done

echo
if [[ $missing -eq 0 ]]; then
  echo "All $ok experimental model(s) present and verified."
  exit 0
fi

cat <<'EOF'
Some experimental weights are absent.

This is expected and is NOT an error for the validated flight pipeline --
ego_raptor.launch.py and the VLM region gate need none of them, and the build
skips the checkpoints directory when it is missing.

These weights are not redistributed from this repository: they are large
binaries belonging to the upstream NavDP and iPlanner projects, and we do not
have the right to mirror them. To use those planners, obtain the checkpoints
from the upstream releases, drop them in

    planner_wrapper/checkpoints/

under the names above, and re-run this script to verify the checksums. The
expected sha256 and sizes are recorded in docs/DEPENDENCIES.md so a file you
obtain elsewhere can be checked against what this workspace expects.

Other model dependencies, for reference:

  Qwen-3.5-2B INT4 engine   built locally from TensorRT-Edge-LLM, device
                            specific -- see docs/INSTALL.md
  Depth-Anything-V2-Small   downloaded automatically by transformers on first
                            run of depth_estimator, no action needed
  RAPTOR policy             lives in the Pixhawk firmware, not in this repo
EOF
exit 0
