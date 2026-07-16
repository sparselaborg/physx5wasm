#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_ROOT="${BUILD_ROOT:-$ROOT_DIR/build}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$ROOT_DIR}"

if (($# == 0)); then
	set -- debug checked profile release
fi

for config in "$@"; do
	case "$config" in
		debug|checked|profile|release) ;;
		*) echo "Unknown PhysX configuration: $config" >&2; exit 2 ;;
	esac

	emcmake cmake \
		-S "$ROOT_DIR/physx" \
		-B "$BUILD_ROOT/emscripten-$config" \
		-G Ninja \
		-DCMAKE_BUILD_TYPE="$config" \
		-DTARGET_BUILD_PLATFORM=emscripten \
		-DPX_OUTPUT_ARCH=wasm32 \
		-DPX_OUTPUT_LIB_DIR="$OUTPUT_ROOT" \
		-DPX_OUTPUT_BIN_DIR="$OUTPUT_ROOT" \
		-DPX_GENERATE_GPU_PROJECTS=OFF \
		-DPX_GENERATE_GPU_PROJECTS_ONLY=OFF \
		-DPX_GENERATE_STATIC_LIBRARIES=ON \
		-DPX_BUILDSNIPPETS=OFF \
		-DPX_BUILDPVDRUNTIME=OFF \
		-DPX_CMAKE_SUPPRESS_REGENERATION=ON
done
