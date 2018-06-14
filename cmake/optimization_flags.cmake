# set common optimization flags

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Detected architecture ${ARCHITECTURE}")

if (${ARCHITECTURE} STREQUAL "armv7l")
    set(USE_SSE2 0 CACHE BOOL "Compile for SSE2 (x86) using -msse2")
    set(USE_SSE4.2 0 CACHE BOOL "Compile for SSE4.2 (x86) using -mpopcnt -msse4.2")
    set(USE_AVX 0 CACHE BOOL "Compile for AVX (x86) using -mavx")
    set(USE_AVX2 0 CACHE BOOL "Compile for AVX2 (x86) using -mavx2")
    set(USE_ARMV7_CA15 1 CACHE BOOL "Compile for ARM V7 Cortex-A15 using -mcpu=cortex-a15 -mfpu=neon")
else ()
    set(USE_SSE2 1 CACHE BOOL "Compile for SSE2 (x86) using -msse2")
    set(USE_SSE4.2 1 CACHE BOOL "Compile for SSE4.2 (x86) using -mpopcnt -msse4.2")
    set(USE_AVX 1 CACHE BOOL "Compile for AVX (x86) using -mavx")
    set(USE_AVX2 1 CACHE BOOL "Compile for AVX2 (x86) using -mavx2")
    set(USE_ARMV7_CA15 0 CACHE BOOL "Compile for ARM V7 Cortex-A15 using -mcpu=cortex-a15 -mfpu=neon")
endif ()
set(UNROLL_LOOPS 1 CACHE BOOL "Compile using -funroll-loops")

if (USE_SSE2)
    add_definitions(-msse2)
endif ()

if (USE_SSE4.2)
    add_definitions(-mpopcnt)
    add_definitions(-msse4.2)
endif ()

if (USE_AVX)
    add_definitions(-mavx)
endif ()

if (USE_AVX2)
    add_definitions(-mavx2)
endif ()

if (USE_ARMV7_CA15)
    add_definitions(-mcpu=cortex-a15 -mfpu=neon)
endif ()

if (UNROLL_LOOPS)
    add_definitions(-funroll-loops)
endif ()
