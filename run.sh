#!/usr/bin/env bash
# Starts Dreamland; installs or catches up first when needed. Arguments go to webbuilder.py
# (e.g. ./run.sh aircraft/albatross, ./run.sh --no-browser).
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -d vendor/inav-jsbsim-bench ] || [ ! -d .venv ] || [ ! -d web/node_modules ] || [ ! -f web/dist/index.html ]; then
    ./setup.sh
else
    uv sync --quiet  # no-op unless uv.lock changed (a pull) or the venv drifted
    # rebuild the browser UI after front-end edits
    if [ -n "$(find web/src web/index.html web/package.json web/vite.config.ts -newer web/dist/index.html -print -quit)" ]; then
        (cd web && npm run build)
    fi
fi

exec uv run python webbuilder.py "$@"
