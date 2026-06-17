# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_laproscopic_grasper_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED laproscopic_grasper_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(laproscopic_grasper_FOUND FALSE)
  elseif(NOT laproscopic_grasper_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(laproscopic_grasper_FOUND FALSE)
  endif()
  return()
endif()
set(_laproscopic_grasper_CONFIG_INCLUDED TRUE)

# output package information
if(NOT laproscopic_grasper_FIND_QUIETLY)
  message(STATUS "Found laproscopic_grasper: 1.0.0 (${laproscopic_grasper_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'laproscopic_grasper' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT laproscopic_grasper_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(laproscopic_grasper_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${laproscopic_grasper_DIR}/${_extra}")
endforeach()
