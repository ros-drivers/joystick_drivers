find_package(SDL2 QUIET)
if(SDL2_FOUND)
  # Some systems (like Ubuntu 18.04) provide CMake "old-style" SDL2_INCLUDE_DIRS
  # and SDL2_LIBRARIES.  In these cases, generate a fake SDL2::SDL2 so that
  # downstreams consumers can just target_link_libraries(SDL2::SDL2)
  if(NOT TARGET SDL2::SDL2)
    add_library(SDL2::SDL2 INTERFACE IMPORTED)
    set_property(TARGET SDL2::SDL2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${SDL2_INCLUDE_DIRS})
    target_link_libraries(SDL2::SDL2 INTERFACE ${SDL2_LIBRARIES})
  endif()
endif()
