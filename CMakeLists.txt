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

# native test runner, skipped when building for the browser
if(NOT EMSCRIPTEN)
  enable_testing()
  add_executable(tests tests/test_main.cpp)
  target_include_directories(tests SYSTEM PRIVATE third_party/doctest)
  target_compile_options(tests PRIVATE -Wall -Wextra)
  add_test(NAME tests COMMAND tests)
endif()
