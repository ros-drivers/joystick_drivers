# Copyright 2021 Intel Corporation
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

find_library(BLUETOOTH_LIB bluetooth)
if(BLUETOOTH_LIB)
  message(STATUS "Found bluetooth library.")
else()
  message(FATAL_ERROR "bluetooth library not found.")
endif()

find_path(BLUETOOTH_INCLUDE_DIR bluetooth/bluetooth.h)
if(BLUETOOTH_LIB)
  message(STATUS "Found bluetooth header.")
else()
  message(FATAL_ERROR "bluetooth header not found.")
endif()

find_library(CWIID_LIB cwiid)
if(CWIID_LIB)
  message(STATUS "Found cwiid library.")
else()
  message(FATAL_ERROR "cwiid library not found.")
endif()

find_path(CWIID_INCLUDE_DIR cwiid.h)
if(CWIID_LIB)
  message(STATUS "Found cwiid header.")
else()
  message(FATAL_ERROR "cwiid header not found.")
endif()

add_library(wiimote::bluetooth INTERFACE IMPORTED)
target_link_libraries(wiimote::bluetooth INTERFACE ${BLUETOOTH_LIB})
target_include_directories(wiimote::bluetooth INTERFACE ${BLUETOOTH_INCLUDE_DIR})

add_library(wiimote::cwiid INTERFACE IMPORTED)
target_link_libraries(wiimote::cwiid INTERFACE ${CWIID_LIB})
target_include_directories(wiimote::cwiid INTERFACE ${CWIID_INCLUDE_DIR})

unset(BLUETOOTH_LIB)
unset(BLUETOOTH_INCLUDE_DIR)
unset(CWIID_LIB)
unset(CWIID_INCLUDE_DIR)
