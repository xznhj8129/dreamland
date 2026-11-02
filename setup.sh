#!/usr/bin/env bash
# The install, one command; safe to re-run (after a pin changes, a pull, or a broken venv).
# Always:
#   - the JSBSim bench (inavbench, used by every sim runner) into vendor/ (gitignored) at the pinned commit
#     (temporary, until it is published)
#   - the Python environment (uv sync; the bench is installed from vendor/inav-jsbsim-bench)
#   - the browser UI (web/dist)
# Optional SITL autopilots:
#   --inav       the INAV fork with the stepped SITL clock into vendor/inav at the pinned commit, built into
#                vendor/inav/build/build_SITL/bin/SITL.elf (incremental; 2 build jobs) with the host toolchain
#                (git, cmake, ninja, ruby and a real GCC, 13 or later recommended: INAV's docs/development/Building SITL.md)
#   --docker     with --inav: build SITL in INAV's Docker image instead (needs Docker access)
#   --px4        PX4 SITL: not supported yet, does nothing
#   --ardupilot  ArduPilot SITL: not supported yet, does nothing
# BENCH_REPO / INAV_REPO override where the pinned commits are fetched from (e.g. a local checkout).
set -euo pipefail

BENCH_REPO="${BENCH_REPO:-https://github.com/xznhj8129/Inav_jsbsim.git}"
BENCH_BRANCH=master
BENCH_COMMIT=11a90d0c7fc12fe287f1606e6e613d3ecdadf83f
INAV_REPO="${INAV_REPO:-https://github.com/xznhj8129/inav.git}"
INAV_BRANCH=frog10
INAV_COMMIT=4a18df1b101d491921464b51ce8c3ddfd47804d0

ROOT="$(cd "$(dirname "$0")" && pwd)"
VENDOR="$ROOT/vendor"
INAV="$VENDOR/inav"

WITH_INAV=0 DOCKER=0 WITH_PX4=0 WITH_ARDUPILOT=0
for arg in "$@"; do
    case "$arg" in
        --inav) WITH_INAV=1 ;;
        --docker) DOCKER=1 ;;
        --px4) WITH_PX4=1 ;;
        --ardupilot) WITH_ARDUPILOT=1 ;;
        *) echo "usage: $0 [--inav [--docker]] [--px4] [--ardupilot]" >&2; exit 2 ;;
    esac
done
if [ "$DOCKER" = 1 ] && [ "$WITH_INAV" = 0 ]; then
    echo "--docker only applies to --inav" >&2; exit 2
fi

# checkout <dir> <repo> <branch> <commit>: clone once, then fetch the branch and check the pinned commit out
checkout() {
    local dir="$1" repo="$2" branch="$3" commit="$4"
    if [ ! -d "$dir/.git" ]; then
        git clone --branch "$branch" --single-branch "$repo" "$dir"
    else
        git -C "$dir" fetch "$repo" "$branch"
    fi
    git -C "$dir" checkout --quiet --detach "$commit"
    echo "$(basename "$dir"): $(git -C "$dir" log -1 --format='%h %s')"
}

mkdir -p "$VENDOR"
checkout "$VENDOR/inav-jsbsim-bench" "$BENCH_REPO" "$BENCH_BRANCH" "$BENCH_COMMIT"

if [ "$WITH_INAV" = 1 ]; then
    checkout "$INAV" "$INAV_REPO" "$INAV_BRANCH" "$INAV_COMMIT"
    if [ "$DOCKER" = 1 ]; then
        # INAV's own SITL build (docs/development/Building in Docker.md): the image's workdir is /src/build
        docker build -t inav-build --build-arg USER_ID="$(id -u)" --build-arg GROUP_ID="$(id -g)" "$INAV"
        mkdir -p "$INAV/build"
        docker run --rm --entrypoint /src/cmake/docker_build_sitl.sh -v "$INAV":/src inav-build
    else
        cmake -S "$INAV" -B "$INAV/build/build_SITL" -DSITL=ON -DDEBUG=ON -DWARNINGS_AS_ERRORS=ON -GNinja  # as cmake/docker_build_sitl.sh
        cmake --build "$INAV/build/build_SITL" --parallel 2
    fi
    ls -l "$INAV/build/build_SITL/bin/SITL.elf"
fi
if [ "$WITH_PX4" = 1 ]; then
    echo "PX4 SITL: not supported yet, nothing installed"
fi
if [ "$WITH_ARDUPILOT" = 1 ]; then
    echo "ArduPilot SITL: not supported yet, nothing installed"
fi

cd "$ROOT"
uv sync
cd web
npm install
npm run build
echo "Done. Start with: ./run.sh"
