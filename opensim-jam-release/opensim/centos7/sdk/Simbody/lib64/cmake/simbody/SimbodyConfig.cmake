# SimbodyConfig.cmake

# This should define the following:
#   Simbody_FOUND - Whether search for Simbody libraries and headers succeeded.
#   Simbody_ROOT_DIR - the installation directory; all the pieces must be
#                      found together
#   Simbody_INCLUDE_DIR - location of Simbody.h
#   Simbody_LIB_DIR     - location of libSimTKsimbody.{a,so,dylib}
#                         or SimTKsimbody.lib
#   Simbody_BIN_DIR     - location of .dll's on Windows
#   Simbody_VIZ_DIR     - location of simbody-visualizer
#   Simbody_LIBRARIES   - suitable for target_link_libraries(); includes
#                         both optimized and debug libraries if both are
#                         available
#   Simbody_STATIC_LIBRARIES - suitable for target_link_libraries(); includes
#                              both optimized and debug static libraries if
#                              both are available
#
# The following variables can be used in your own project so that your
# project's Doxygen documentation can link with Simbody's. These variables are
# only defined if Doxygen documentation is installed.
#   Simbody_DOXYGEN_DIR     - Directory containing Doxygen API documentation.
#   Simbody_DOXYGEN_TAGFILE - Path to SimbodyDoxygenTagFile.
#
# For example, if you're configuring your Doxyfile using CMake's
# configure_file, your Doxyfile.in file (to be configured) could contain
# (without the backslashes):
#
#   TAGFILES = "\@Simbody_DOXYGEN_TAGFILE\@=\@Simbody_DOXYGEN_DIR\@"

# To make the Simbody installation relocatable:

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SimbodyConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# Watch out for spaces in pathnames -- must quote.
set_and_check(Simbody_ROOT_DIR
              "${PACKAGE_PREFIX_DIR}")

set_and_check(Simbody_INCLUDE_DIR
              "${PACKAGE_PREFIX_DIR}/include/simbody")

set_and_check(Simbody_LIB_DIR
              "${PACKAGE_PREFIX_DIR}/lib64")

list(APPEND Simbody_BIN_DIR
            "${PACKAGE_PREFIX_DIR}/bin")

list(APPEND Simbody_VIZ_DIR
            "${PACKAGE_PREFIX_DIR}/libexec/simbody")

list(APPEND Simbody_CFLAGS
            -I"${PACKAGE_PREFIX_DIR}/include/simbody")

list(APPEND Simbody_LDFLAGS
            -L"${PACKAGE_PREFIX_DIR}/lib64")

if (NOT "SimbodyDoxygenTagfile" STREQUAL "")
    # Must check tagfile variable, since the doxygen install dir is created
    # even if Doxygen documentation is not installed.
    set(temp_doxygen_dir "${PACKAGE_PREFIX_DIR}/share/doc/simbody/api")
    set(temp_tagfile_path
        "${temp_doxygen_dir}/SimbodyDoxygenTagfile")
    if (EXISTS "${temp_tagfile_path}")
        set(Simbody_DOXYGEN_DIR "${temp_doxygen_dir}")
        set(Simbody_DOXYGEN_TAGFILE "${temp_tagfile_path}")
    endif()
    unset(temp_doxygen_dir)
    unset(temp_tagfile_path)
endif()

# Our library dependencies (contains definitions for IMPORTED targets)
include("${CMAKE_CURRENT_LIST_DIR}/SimbodyTargets.cmake")


# Create "fake" IMPORTED targets to represent the pre-built platform libraries
# that Simbody usually carries along on Windows.
# When CMake sees that a target links to, e.g., the blas target, CMake will
# use the appropriate library paths below.
set(SIMBODY_WAS_BUILT_USING_OTHER_LAPACK "")
if(WIN32 AND NOT SIMBODY_WAS_BUILT_USING_OTHER_LAPACK)
    add_library(blas SHARED IMPORTED)
    set_target_properties(blas PROPERTIES
        IMPORTED_IMPLIB "${PACKAGE_PREFIX_DIR}/lib64/libblas.lib"
        IMPORTED_LOCATION "${PACKAGE_PREFIX_DIR}/bin/libblas.dll"
        )

    add_library(lapack SHARED IMPORTED)
    set_target_properties(lapack PROPERTIES
        IMPORTED_IMPLIB "${PACKAGE_PREFIX_DIR}/lib64/liblapack.lib"
        IMPORTED_LOCATION "${PACKAGE_PREFIX_DIR}/bin/liblapack.dll"
        # lapack depends on blas:
        INTERFACE_LINK_LIBRARIES blas
        )
endif()


# These are IMPORTED targets created by SimbodyTargets.cmake
if(TRUE)
    set(Simbody_LIBRARIES SimTKcommon SimTKmath SimTKsimbody)
else()
    set(Simbody_LIBRARIES Simbody_LIBRARIES-NOTFOUND)
endif()
if(FALSE) # this is ON if static libraries were built
    set(Simbody_STATIC_LIBRARIES SimTKcommon_static SimTKmath_static SimTKsimbody_static)
else()
    set(Simbody_STATIC_LIBRARIES Simbody_STATIC_LIBRARIES-NOTFOUND)
endif()
mark_as_advanced(Simbody_LIBRARIES Simbody_STATIC_LIBRARIES)

check_required_components(Simbody)
