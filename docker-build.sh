#!/usr/bin/env bash

set -euo pipefail

HOST_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(mktemp -d /tmp/physx-source.XXXXXX)"

cp -a "$HOST_ROOT/physx" "$SOURCE_ROOT/physx"
cp "$HOST_ROOT/generate.sh" "$HOST_ROOT/make.sh" "$SOURCE_ROOT/"

export BUILD_ROOT=/tmp/physx-build
export OUTPUT_ROOT=/tmp/physx-output
export DIST_ROOT="${DIST_ROOT:-/tmp/physx-dist}"

bash "$SOURCE_ROOT/generate.sh" "$@"
bash "$SOURCE_ROOT/make.sh" "$@"
