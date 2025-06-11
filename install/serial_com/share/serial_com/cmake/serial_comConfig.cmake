# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_serial_com_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED serial_com_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(serial_com_FOUND FALSE)
  elseif(NOT serial_com_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(serial_com_FOUND FALSE)
  endif()
  return()
endif()
set(_serial_com_CONFIG_INCLUDED TRUE)

# output package information
if(NOT serial_com_FIND_QUIETLY)
  message(STATUS "Found serial_com: 0.0.0 (${serial_com_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'serial_com' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${serial_com_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(serial_com_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${serial_com_DIR}/${_extra}")
endforeach()
