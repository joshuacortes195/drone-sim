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

# warning flags, msvc on windows uses its own spelling
if(MSVC)
  set(WARN_FLAGS /W4)
else()
  set(WARN_FLAGS -Wall -Wextra)
endif()

# the drone physics, shared by the tests, tools and later the browser build
add_library(core core/quad.cpp core/controller.cpp core/map.cpp core/planner.cpp core/route.cpp core/sensors.cpp core/estimator.cpp core/lidar.cpp core/wind.cpp)
target_include_directories(core PUBLIC core)
target_compile_options(core PRIVATE ${WARN_FLAGS})

# native test runner, skipped when building for the browser
if(NOT EMSCRIPTEN)
  enable_testing()
  add_executable(tests tests/test_main.cpp tests/test_quad.cpp tests/test_controller.cpp tests/test_planning.cpp tests/test_route.cpp tests/test_sensors.cpp tests/test_estimator.cpp tests/test_lidar.cpp tests/test_wind.cpp)
  target_include_directories(tests SYSTEM PRIVATE third_party/doctest)
  target_include_directories(tests PRIVATE tools)
  target_compile_options(tests PRIVATE ${WARN_FLAGS})
  target_link_libraries(tests PRIVATE core)
  add_test(NAME tests COMMAND tests)

  # prints the physics numbers for results/physics.md
  add_executable(physics_report tools/physics_report.cpp)
  target_compile_options(physics_report PRIVATE ${WARN_FLAGS})
  target_link_libraries(physics_report PRIVATE core)

  # flies the test route and prints step responses and tracking error for results/control.md
  add_executable(fly_route tools/fly_route.cpp)
  target_compile_options(fly_route PRIVATE ${WARN_FLAGS})
  target_link_libraries(fly_route PRIVATE core)

  # plans across many seeded cities and prints timing and path length for results/planning.md
  add_executable(plan_report tools/plan_report.cpp)
  target_compile_options(plan_report PRIVATE ${WARN_FLAGS})
  target_link_libraries(plan_report PRIVATE core)

  # flies past a box the drone doesn't know about, with and without the lidar, for results/lidar.md
  add_executable(lidar_report tools/lidar_report.cpp)
  target_compile_options(lidar_report PRIVATE ${WARN_FLAGS})
  target_link_libraries(lidar_report PRIVATE core)
endif()

# browser build: the core plus the bindings, written straight into the web app as an es module and a .wasm file
if(EMSCRIPTEN)
  add_executable(drone wasm/bindings.cpp)
  target_link_libraries(drone PRIVATE core)
  target_compile_options(drone PRIVATE ${WARN_FLAGS})
  # fixed size memory that never grows, so the views js holds into it never go dead.
  # 64 MB leaves room for a* on the 576,000 cell planning grid
  target_link_options(drone PRIVATE
    -sMODULARIZE=1 -sEXPORT_ES6=1 -sENVIRONMENT=web
    -sINITIAL_MEMORY=64MB -sALLOW_MEMORY_GROWTH=0
    -sEXPORTED_RUNTIME_METHODS=HEAPF64 -sFILESYSTEM=0)
  set_target_properties(drone PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/web/src/wasm)
endif()
