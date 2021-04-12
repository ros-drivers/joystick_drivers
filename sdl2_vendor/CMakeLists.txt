cmake_minimum_required(VERSION 3.5)
project(sdl2_vendor)

option(FORCE_BUILD_VENDOR_PKG
  "Build SDL2 from source, even if system-installed package is available"
  OFF)

find_package(ament_cmake REQUIRED)

if(NOT FORCE_BUILD_VENDOR_PKG)
  list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
  find_package(sdl2_custom QUIET)
endif()

if(NOT SDL2_FOUND)
  include(ExternalProject)
  ExternalProject_Add(SDL2-2.0.14
    PREFIX SDL2-2.0.12
    URL https://www.libsdl.org/release/SDL2-2.0.14.tar.gz
    URL_MD5 76ed4e6da9c07bd168b2acd9bfefab1b
    CMAKE_ARGS
      -DBUILD_SHARED_LIBS=ON
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/SDL2
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
      -DSDL_ATOMIC=OFF
      -DSDL_AUDIO=OFF
      -DSDL_VIDEO=ON     # On, needed to link on Windows
      -DSDL_RENDER=ON    # On, needed to link on Windows
      -DSDL_EVENTS=ON    # On, dependency of joystick
      -DSDL_JOYSTICK=ON  # On, we are using this
      -DSDL_HAPTIC=ON    # On, we are using this
      -DSDL_POWER=ON     # On, dependency of joystick
      -DSDL_THREADS=OFF
      -DSDL_TIMERS=ON    # On, we need timers to work
      -DSDL_FILE=ON      # On, we need to build on macOS
      -DSDL_LOADSO=ON    # On, needed to link on Windows
      -DSDL_CPUINFO=OFF
      -DSDL_FILESYSTEM=OFF
      -DSDL_DLOPEN=ON   # On, dependency of joystick
      -DSDL_SENSOR=ON   # On, dependency of joystick
    INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/SDL2"
    PATCH_COMMAND
      ${CMAKE_COMMAND} -E chdir <SOURCE_DIR> git apply -p1 --ignore-space-change --whitespace=nowarn ${CMAKE_CURRENT_SOURCE_DIR}/sdl2-windows-add-vcruntime.patch
    )

  # The external project will install to the build folder, but we'll install that on make install.
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/SDL2/ DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "sdl2_vendor-extras.cmake"
)
