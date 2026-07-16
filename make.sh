#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_ROOT="${BUILD_ROOT:-$ROOT_DIR/build}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$ROOT_DIR}"
DIST_ROOT="${DIST_ROOT:-$ROOT_DIR/dist}"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)}"

if (($# == 0)); then
	set -- debug checked profile release
fi

for config in "$@"; do
	case "$config" in
		debug|checked|profile|release) ;;
		*) echo "Unknown PhysX configuration: $config" >&2; exit 2 ;;
	esac

	cmake --build "$BUILD_ROOT/emscripten-$config" --parallel "$JOBS"

	source_dir="$OUTPUT_ROOT/bin/wasm32/$config"
	destination_dir="$DIST_ROOT/$config"
	libraries=(
		libPhysXCharacterKinematic_static.a
		libPhysXCommon_static.a
		libPhysXCooking_static.a
		libPhysXExtensions_static.a
		libPhysXFoundation_static.a
		libPhysXPvdSDK.a
		libPhysXVehicle_static.a
		libPhysX_static.a
	)

	cmake -E make_directory "$destination_dir"
	for library in "${libraries[@]}"; do
		cmake -E copy_if_different "$source_dir/$library" "$destination_dir/$library"
	done
done
