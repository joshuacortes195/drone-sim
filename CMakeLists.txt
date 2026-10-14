cmake_minimum_required(VERSION 3.20)
project(drone_sim CXX)

# c++17 everywhere, no compiler specific extensions
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# default to an optimized build so tests and benchmarks run at real speed
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# the drone physics, shared by the tests, tools and later the browser build
add_library(core core/quad.cpp)
target_include_directories(core PUBLIC core)
target_compile_options(core PRIVATE -Wall -Wextra)

# native test runner, skipped when building for the browser
if(NOT EMSCRIPTEN)
  enable_testing()
  add_executable(tests tests/test_main.cpp tests/test_quad.cpp)
  target_include_directories(tests SYSTEM PRIVATE third_party/doctest)
  target_compile_options(tests PRIVATE -Wall -Wextra)
  target_link_libraries(tests PRIVATE core)
  add_test(NAME tests COMMAND tests)

  # prints the physics numbers for results/physics.md
  add_executable(physics_report tools/physics_report.cpp)
  target_compile_options(physics_report PRIVATE -Wall -Wextra)
  target_link_libraries(physics_report PRIVATE core)
endif()
