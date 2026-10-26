cmake_minimum_required(VERSION 3.20)

# Web build: find the Emscripten toolchain next to emcc on PATH, so the `web`
# preset works without wrapping CMake in emcmake.
option(SLIDE_WEB "Build the visualizer for the browser with Emscripten" OFF)
if(SLIDE_WEB AND NOT CMAKE_TOOLCHAIN_FILE)
  find_program(SLIDE_EMCC emcc REQUIRED)
  file(REAL_PATH "${SLIDE_EMCC}" _emcc)
  get_filename_component(_emcc_dir "${_emcc}" DIRECTORY)
  foreach(_dir "${_emcc_dir}" "${_emcc_dir}/../libexec")  # emsdk, Homebrew
    if(EXISTS "${_dir}/cmake/Modules/Platform/Emscripten.cmake")
      set(CMAKE_TOOLCHAIN_FILE "${_dir}/cmake/Modules/Platform/Emscripten.cmake")
    endif()
  endforeach()
endif()

project(Slide VERSION 1.0.0 DESCRIPTION "Real-time vehicle dynamics engine" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(SLIDE_BUILD_APP "Build the raylib visualizer" ON)
option(SLIDE_BUILD_TESTS "Build the physics test suite" ON)
option(SLIDE_FETCH_RAYLIB "Download raylib when it is not installed" ON)

add_library(slide_warnings INTERFACE)
if(MSVC)
  target_compile_options(slide_warnings INTERFACE /W4)
else()
  target_compile_options(slide_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow)
endif()

# Physics core: no dependencies beyond the standard library.
add_library(slide_physics STATIC
  src/physics/Autopilot.cpp
  src/physics/DriverAids.cpp
  src/physics/Drivetrain.cpp
  src/physics/Engine.cpp
  src/physics/Suspension.cpp
  src/physics/Telemetry.cpp
  src/physics/Tire.cpp
  src/physics/Track.cpp
  src/physics/Vehicle.cpp
)
target_include_directories(slide_physics PUBLIC include)
target_link_libraries(slide_physics PRIVATE slide_warnings)

# Headless test stand: engine dyno, launch, braking, skidpad, laps, benchmark.
if(NOT EMSCRIPTEN)
  add_executable(slide-bench src/bench/main.cpp)
  target_link_libraries(slide-bench PRIVATE slide_physics slide_warnings)
endif()

if(SLIDE_BUILD_TESTS)
  enable_testing()
  add_executable(slide-tests
    tests/main.cpp
    tests/DrivetrainTests.cpp
    tests/EngineTests.cpp
    tests/TireTests.cpp
    tests/VehicleTests.cpp
  )
  target_link_libraries(slide-tests PRIVATE slide_physics slide_warnings)
  add_test(NAME physics COMMAND slide-tests)
endif()

if(SLIDE_BUILD_APP)
  if(EMSCRIPTEN)
    set(PLATFORM Web CACHE STRING "" FORCE)  # raylib's own option
  else()
    find_package(raylib 5.0 QUIET)
  endif()
  if(NOT raylib_FOUND AND SLIDE_FETCH_RAYLIB)
    include(FetchContent)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(raylib
      URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz)
    FetchContent_MakeAvailable(raylib)
  endif()

  if(TARGET raylib)
    add_executable(slide
      src/app/main.cpp
      src/app/Effects.cpp
      src/app/Hud.cpp
      src/app/WorldRenderer.cpp
    )
    target_link_libraries(slide PRIVATE slide_physics slide_warnings raylib)
    if(EMSCRIPTEN)
      # Produces a static site: index.html, index.js, index.wasm, index.data.
      set_target_properties(slide PROPERTIES OUTPUT_NAME index SUFFIX ".html")
      target_link_options(slide PRIVATE
        -sUSE_GLFW=3
        -sALLOW_MEMORY_GROWTH=1
        --shell-file ${CMAKE_SOURCE_DIR}/src/app/web/shell.html
        --preload-file ${CMAKE_SOURCE_DIR}/assets/fonts/DejaVuSansMono.ttf@/fonts/DejaVuSansMono.ttf)
      set_property(TARGET slide APPEND PROPERTY LINK_DEPENDS ${CMAKE_SOURCE_DIR}/src/app/web/shell.html)
    endif()
  else()
    message(WARNING "raylib not found and SLIDE_FETCH_RAYLIB is OFF; skipping the visualizer")
  endif()
endif()
