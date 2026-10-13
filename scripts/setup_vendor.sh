#!/usr/bin/env bash
# Temporary, until both are published: puts the INAV JSBSim bench and the INAV fork with the stepped SITL clock in
# vendor/ (gitignored) at the pinned commits, and builds INAV SITL into vendor/inav/build/build_SITL/bin/SITL.elf.
# Re-run after a pin changes. Run it before `uv sync`: the bench is installed from vendor/inav-jsbsim-bench.
#
# Usage: scripts/setup_vendor.sh           build SITL in INAV's Docker image (needs git and Docker)
#        scripts/setup_vendor.sh --host    build SITL with the host toolchain (git, cmake, ninja, ruby and a real GCC,
#                                          13 or later recommended: INAV's docs/development/Building SITL.md)
# BENCH_REPO / INAV_REPO override where the pinned commits are fetched from (e.g. a local checkout).
set -euo pipefail

BENCH_REPO="${BENCH_REPO:-https://github.com/xznhj8129/Inav_jsbsim.git}"
BENCH_BRANCH=master
BENCH_COMMIT=11a90d0c7fc12fe287f1606e6e613d3ecdadf83f
INAV_REPO="${INAV_REPO:-https://github.com/xznhj8129/inav.git}"
INAV_BRANCH=frog10
INAV_COMMIT=4a18df1b101d491921464b51ce8c3ddfd47804d0

VENDOR="$(cd "$(dirname "$0")/.." && pwd)/vendor"
INAV="$VENDOR/inav"

case "${1:-docker}" in
    docker|--host) BUILD="${1:-docker}" ;;
    *) echo "usage: $0 [--host]" >&2; exit 2 ;;
esac

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
checkout "$INAV" "$INAV_REPO" "$INAV_BRANCH" "$INAV_COMMIT"

if [ "$BUILD" = docker ]; then
    # INAV's own SITL build (docs/development/Building in Docker.md): the image's workdir is /src/build
    docker build -t inav-build --build-arg USER_ID="$(id -u)" --build-arg GROUP_ID="$(id -g)" "$INAV"
    mkdir -p "$INAV/build"
    docker run --rm --entrypoint /src/cmake/docker_build_sitl.sh -v "$INAV":/src inav-build
else
    cmake -S "$INAV" -B "$INAV/build/build_SITL" -DSITL=ON -DDEBUG=ON -DWARNINGS_AS_ERRORS=ON -GNinja  # as cmake/docker_build_sitl.sh
    cmake --build "$INAV/build/build_SITL"
fi
ls -l "$INAV/build/build_SITL/bin/SITL.elf"
