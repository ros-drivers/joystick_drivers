# Find the spnav library and header.
#
# Sets the usual variables expected for find_package scripts:
#
# spnav_INCLUDE_DIR - header location
# spnav_LIBRARIES - library to link against
# spnav_FOUND - true if pugixml was found.

if(UNIX)

  find_path(spnav_INCLUDE_DIR spnav.h)

  find_library(spnav_LIBRARY
    NAMES
    spnav libspnav
)

# Support the REQUIRED and QUIET arguments, and set spnav_FOUND if found.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SPNAV DEFAULT_MSG
  spnav_LIBRARY
  spnav_INCLUDE_DIR)

if(spnav_FOUND)
  set(spnav_LIBRARIES ${spnav_LIBRARY})
endif()

mark_as_advanced(
  spnav_LIBRARY
  spnav_INCLUDE_DIR)

endif()
