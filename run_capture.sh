#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
if [[ ! -x .venv/bin/python ]]; then
    echo 'Run ./setup.sh first.' >&2
    exit 1
fi
unset PYTHONPATH PYTHONHOME
exec .venv/bin/python -s -B -m datacapture "$@"
