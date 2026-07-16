## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the conditions in LICENSE.md
## are met.
##
## Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

# Keep the warning policy and compatibility suppressions from the established
# WASM build. Emscripten reports itself as Clang.
SET(CLANG_WARNINGS "-ferror-limit=0 -Wall -Wextra -Werror -Wstrict-aliasing=2 -Weverything \
    -Wno-alloca \
    -Wno-atomic-implicit-seq-cst \
    -Wno-c++98-compat-pedantic \
    -Wno-c++98-compat \
    -Wno-cast-align \
    -Wno-conversion \
    -Wno-covered-switch-default \
    -Wno-deprecated \
    -Wno-documentation-deprecated-sync \
    -Wno-documentation-unknown-command \
    -Wno-exit-time-destructors \
    -Wno-extra-semi-stmt \
    -Wno-float-equal \
    -Wno-format-nonliteral \
    -Wno-global-constructors \
    -Wno-gnu-anonymous-struct \
    -Wno-implicit-fallthrough \
    -Wno-inconsistent-missing-destructor-override \
    -Wno-inconsistent-missing-override \
    -Wno-missing-noreturn \
    -Wno-missing-include-dirs \
    -Wno-missing-prototypes \
    -Wno-missing-variable-declarations \
    -Wno-newline-eof \
    -Wno-non-virtual-dtor \
    -Wno-nrvo \
    -Wno-old-style-cast \
    -Wno-padded \
    -Wno-reserved-id-macro \
    -Wno-shadow \
    -Wno-suggest-destructor-override \
    -Wno-suggest-override \
    -Wno-switch-enum \
    -Wno-undef \
    -Wno-undefined-reinterpret-cast \
    -Wno-unknown-pragmas \
    -Wno-unknown-warning-option \
    -Wno-unreachable-code \
    -Wno-unique-object-duplication \
    -Wno-unused-function \
    -Wno-unused-local-typedef \
    -Wno-unused-macros \
    -Wno-unused-member-function \
    -Wno-unused-private-field \
    -Wno-used-but-marked-unused \
    -Wno-weak-template-vtables \
    -Wno-weak-vtables \
    -Wno-zero-as-null-pointer-constant \
    -Wno-reserved-identifier \
    -Wno-undefined-func-template \
    -Wno-cast-qual \
    -Wno-double-promotion \
    -Wno-extra-semi \
    -Wno-conditional-uninitialized \
    -Wno-unused-parameter \
    -Wno-unsafe-buffer-usage \
    -Wno-unsupported-floating-point-opt \
    -Wno-unused-but-set-variable \
    -Wno-unused-variable \
    -Wno-switch-default \
    -Wno-invalid-offsetof \
    -Wno-lifetime-safety \
    -Wno-lifetime-safety-cross-tu-constructor-suggestions \
    -Wno-lifetime-safety-cross-tu-suggestions \
    -Wno-lifetime-safety-intra-tu-constructor-suggestions \
    -Wno-lifetime-safety-intra-tu-suggestions \
    -Wno-format \
    -Wno-nan-infinity-disabled")

SET(PHYSX_WASM_PTHREAD_FLAGS "-pthread -DWASMPTHREAD")
SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -fstrict-aliasing -fvisibility=hidden -ffp-exception-behavior=maytrap ${CLANG_WARNINGS}" CACHE INTERNAL "PhysX CXX")

# Deliberately do not enable fast-math: it changes PhysX behavior under WASM.
SET(PHYSX_CXX_FLAGS_DEBUG   "-O0 -msimd128 -mavx -g3 -gdwarf-2 ${PHYSX_WASM_PTHREAD_FLAGS}" CACHE INTERNAL "PhysX Debug CXX Flags")
SET(PHYSX_CXX_FLAGS_CHECKED "-O3 -msimd128 -mavx -g3 -gdwarf-2 ${PHYSX_WASM_PTHREAD_FLAGS}" CACHE INTERNAL "PhysX Checked CXX Flags")
SET(PHYSX_CXX_FLAGS_PROFILE "-O3 -msimd128 -mavx ${PHYSX_WASM_PTHREAD_FLAGS}" CACHE INTERNAL "PhysX Profile CXX Flags")
SET(PHYSX_CXX_FLAGS_RELEASE "-O3 -msimd128 -mavx ${PHYSX_WASM_PTHREAD_FLAGS}" CACHE INTERNAL "PhysX Release CXX Flags")

SET(CMAKE_CXX_FLAGS ${PHYSX_CXX_FLAGS})
SET(CMAKE_CXX_FLAGS_DEBUG   ${PHYSX_CXX_FLAGS_DEBUG})
SET(CMAKE_CXX_FLAGS_CHECKED ${PHYSX_CXX_FLAGS_CHECKED})
SET(CMAKE_CXX_FLAGS_PROFILE ${PHYSX_CXX_FLAGS_PROFILE})
SET(CMAKE_CXX_FLAGS_RELEASE ${PHYSX_CXX_FLAGS_RELEASE})

# Keep the archive names consumed by the existing WASM integration. The
# architecture is already represented by the bin/wasm32 directory.
SET(CMAKE_DEBUG_POSTFIX "")
SET(CMAKE_CHECKED_POSTFIX "")
SET(CMAKE_PROFILE_POSTFIX "")
SET(CMAKE_RELEASE_POSTFIX "")

IF(PUBLIC_RELEASE)
	SET(PUBLIC_RELEASE_FLAG "PX_PUBLIC_RELEASE=1")
ELSE()
	SET(PUBLIC_RELEASE_FLAG "PX_PUBLIC_RELEASE=0")
ENDIF()

SET(NVTX_FLAG "PX_NVTX=0")
SET(PX_SUPPORT_OMNI_PVD_FLAG "PX_SUPPORT_OMNI_PVD=0")
SET(DISABLE_CUDA_DEF "DISABLE_CUDA_PHYSX")
SET(PHYSX_LINUX_COMPILE_DEFS "${PHYSX_AUTOBUILD};${PUBLIC_RELEASE_FLAG};${DISABLE_CUDA_DEF}" CACHE INTERNAL "Base PhysX preprocessor definitions")
SET(PHYSX_LINUX_DEBUG_COMPILE_DEFS   "PX_DEBUG=1;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=0;${PX_SUPPORT_OMNI_PVD_FLAG}" CACHE INTERNAL "Debug PhysX preprocessor definitions")
SET(PHYSX_LINUX_CHECKED_COMPILE_DEFS "NDEBUG;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=0;${PX_SUPPORT_OMNI_PVD_FLAG}" CACHE INTERNAL "Checked PhysX preprocessor definitions")
SET(PHYSX_LINUX_PROFILE_COMPILE_DEFS "NDEBUG;PX_PROFILE=1;${NVTX_FLAG};PX_SUPPORT_PVD=0;${PX_SUPPORT_OMNI_PVD_FLAG}" CACHE INTERNAL "Profile PhysX preprocessor definitions")
SET(PHYSX_LINUX_RELEASE_COMPILE_DEFS "NDEBUG;PX_SUPPORT_PVD=0;PX_SUPPORT_OMNI_PVD=0" CACHE INTERNAL "Release PhysX preprocessor definitions")

INCLUDE(PhysXFoundation.cmake)
INCLUDE(LowLevel.cmake)
INCLUDE(LowLevelAABB.cmake)
INCLUDE(LowLevelDynamics.cmake)
INCLUDE(PhysX.cmake)
INCLUDE(PhysXCharacterKinematic.cmake)
INCLUDE(PhysXCommon.cmake)
INCLUDE(PhysXCooking.cmake)
INCLUDE(PhysXExtensions.cmake)
INCLUDE(PhysXVehicle.cmake)
INCLUDE(SceneQuery.cmake)
INCLUDE(SimulationController.cmake)
INCLUDE(PhysXPvdSDK.cmake)
INCLUDE(PhysXTask.cmake)

SET_TARGET_PROPERTIES(PhysXPvdSDK PROPERTIES
	ARCHIVE_OUTPUT_NAME_DEBUG PhysXPvdSDK
	ARCHIVE_OUTPUT_NAME_CHECKED PhysXPvdSDK
	ARCHIVE_OUTPUT_NAME_PROFILE PhysXPvdSDK
	ARCHIVE_OUTPUT_NAME_RELEASE PhysXPvdSDK)

SET(PHYSXDISTRO_LIBS PhysXFoundation PhysX PhysXCharacterKinematic PhysXPvdSDK PhysXCommon PhysXCooking PhysXExtensions PhysXVehicle)
