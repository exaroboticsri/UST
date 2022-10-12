# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_velocity_smoother_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED velocity_smoother_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(velocity_smoother_FOUND FALSE)
  elseif(NOT velocity_smoother_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(velocity_smoother_FOUND FALSE)
  endif()
  return()
endif()
set(_velocity_smoother_CONFIG_INCLUDED TRUE)

# output package information
if(NOT velocity_smoother_FIND_QUIETLY)
  message(STATUS "Found velocity_smoother: 0.14.0 (${velocity_smoother_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'velocity_smoother' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${velocity_smoother_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(velocity_smoother_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${velocity_smoother_DIR}/${_extra}")
endforeach()
