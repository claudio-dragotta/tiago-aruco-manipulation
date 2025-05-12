# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_progetto_tiago_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED progetto_tiago_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(progetto_tiago_FOUND FALSE)
  elseif(NOT progetto_tiago_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(progetto_tiago_FOUND FALSE)
  endif()
  return()
endif()
set(_progetto_tiago_CONFIG_INCLUDED TRUE)

# output package information
if(NOT progetto_tiago_FIND_QUIETLY)
  message(STATUS "Found progetto_tiago: 0.0.1 (${progetto_tiago_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'progetto_tiago' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${progetto_tiago_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(progetto_tiago_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${progetto_tiago_DIR}/${_extra}")
endforeach()
