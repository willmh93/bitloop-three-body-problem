cmake_minimum_required(VERSION 3.21)
project(Mandelbrot LANGUAGES C CXX)
include(CheckCXXCompilerFlag)
include("${CMAKE_CURRENT_LIST_DIR}/tooling/cmake/findBitloop.cmake")


# ---------------------------------  flags  ----------------------------------

# if this Cmake project open as root, BITLOOP_DEV_MODE overrides child projects
# set(BITLOOP_DEV_MODE TRUE)

# ------------------  configure project hierarchy/sources  -------------------

file(GLOB SIM_SOURCES CONFIGURE_DEPENDS "ThreeBodyProblem/*.cpp" "ThreeBodyProblem/*.h")

# Create new project
bitloop_new_project(ThreeBodyProblem ${SIM_SOURCES})
bitloop_finalize()

# fast math
#if (NOT CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
#	if (MSVC)
#		target_compile_options(ThreeBodyProblem PRIVATE /arch:AVX2 /fp:fast)
#	else() # GCC/Clang
#		target_compile_options(ThreeBodyProblem PRIVATE -mavx2 -mfma -ffp-contract=fast -ffast-math -fno-math-errno)
#	endif()
#else()
#	target_compile_options(ThreeBodyProblem PRIVATE -O3 -flto -msimd128 -mrelaxed-simd -ffast-math -ffp-contract=fast -fno-math-errno)
#endif()
