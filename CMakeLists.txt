cmake_minimum_required(VERSION 3.21)
project(ThreeBodyProblem LANGUAGES CXX)

if (NOT TARGET bitloop::bitloop)
  find_package(bitloop CONFIG REQUIRED)
endif()

file(GLOB SIM_SOURCES CONFIGURE_DEPENDS "ThreeBodyProblem/*.cpp" "ThreeBodyProblem/*.h")

# Create new project
bitloop_new_project(ThreeBodyProblem ${SIM_SOURCES})

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

# Add dependency projects
#bitloop_add_dependency(ThreeBodyProblem  "./path/to/project")

bitloop_finalize()
