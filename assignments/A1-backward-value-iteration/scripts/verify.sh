#!/usr/bin/env bash
# Math + code verification without ROS: unit tests, 3 difficulty demos with
# optimality certificate, 6-run trade-off table. Usage: ./scripts/verify.sh
set -euo pipefail
source "$(dirname "$0")/ros_env.sh" >/dev/null
cd "$BVI_PROJECT"
# ROS ships pytest plugins (launch_testing) that break on pytest 9; ours need none.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
for level in easy medium hard; do
    echo; echo "===== $level ====="
    python standalone/run_demo.py --difficulty "$level" --compare-bvi --save --no-show
done
echo
python standalone/tradeoff.py
