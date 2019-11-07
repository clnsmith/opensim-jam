#CMakeLists.txt for JAM_Plugin
#===============================================================================
#
#
#-------------------------------------------------------------------------------
# Author: Colin Smith
# Date: 1/27/2017
#===============================================================================
set(PLUGIN_NAME "jam_plugin"
    CACHE STRING "Name of shared library to create")

#Specify Files
file(GLOB SOURCE_FILES *.cpp *.c)
file(GLOB INCLUDE_FILES *.h)

#Specify Plugin Details
add_library(${PLUGIN_NAME} SHARED ${SOURCE_FILES} ${INCLUDE_FILES})
set_target_properties(${PLUGIN_NAME} PROPERTIES DEBUG_POSTFIX "_d")
#Specify Libraries to Link

target_link_libraries(${PLUGIN_NAME} ${OpenSim_LIBRARIES})
target_link_libraries(${PLUGIN_NAME} ${JAM_TOOLS_NAME})

SET_TARGET_PROPERTIES (${PLUGIN_NAME} PROPERTIES FOLDER jam_plugin)

# Find dependencies
# -----------------------
set(special_ext_lib "HDF5;")

set(JAM_dependencies_install_dir "${JAM_OPENSIM_DIR}/../install-dependencies" CACHE PATH "Location of install-dependencies directory" FORCE)

FILE(GLOB EXTERNAL_LIB_NAMES RELATIVE "${JAM_dependencies_install_dir}/" "${JAM_dependencies_install_dir}/*")

FOREACH(external_lib_name ${EXTERNAL_LIB_NAMES})
  if(external_lib_name IN_LIST special_ext_lib)
    continue()
  endif()

  find_library(external_lib_debug "${external_lib_name}_d" HINTS "${JAM_dependencies_install_dir}/${external_lib_name}/")
  find_library(external_lib "${external_lib_name}" HINTS "${JAM_dependencies_install_dir}/${external_lib_name}/")

  if(${external_lib} STREQUAL "external_lib-NOTFOUND")
      message(WARNING "Couldn't find the ${external_lib_name} library")
  elseif(${external_lib_debug} STREQUAL "external_lib_debug-NOTFOUND")
      message(WARNING "Couldn't find the ${external_lib_name} debug library")
  else()
    message(STATUS "Found ${external_lib_name} at: ${external_lib}")
  endif()

  if(external_lib AND external_lib_debug)
    set(EXT_LIB optimized ${external_lib} debug ${external_lib_debug})
  elseif(external_lib)
    set(EXT_LIB ${external_lib})
  elseif(external_lib_debug)
    set(EXT_LIB ${external_lib_debug})
  endif()

  target_link_libraries(${PLUGIN_NAME} ${EXT_LIB})

  unset(external_lib_debug)
  unset(external_lib)
  unset(EXT_LIB)
ENDFOREACH(external_lib_name)

# Find Special External Libraries
#--------------------------------
#HDF5
if (WIN32 OR MSVC)

  find_library(HDF5_hl_cpp_lib_debug "libhdf5_hl_cpp_D.lib" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")
  find_library(HDF5_hl_cpp_lib "libhdf5_hl_cpp.lib" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")

  find_library(HDF5_cpp_lib_debug "libhdf5_cpp_D.lib" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")
  find_library(HDF5_cpp_lib "libhdf5_cpp.lib" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")

  find_library(HDF5_lib_debug "libhdf5_D.lib" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")
  find_library(HDF5_lib "libhdf5.lib" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")

elseif (UNIX)
  find_library(HDF5_hl_cpp_lib_debug "libhdf5_hl_cpp_debug.a" HINTS "${JAM_OPENSIM_DIR}/../debug-install-dependencies/HDF5/lib/")
  find_library(HDF5_hl_cpp_lib "libhdf5_hl_cpp.a" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")

  find_library(HDF5_cpp_lib_debug "libhdf5_cpp_debug.a" HINTS "${JAM_OPENSIM_DIR}/../debug-install-dependencies/HDF5/lib/")
  find_library(HDF5_cpp_lib "libhdf5_cpp.a" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")

  find_library(HDF5_lib_debug "libhdf5_debug.a" HINTS "${JAM_OPENSIM_DIR}/../debug-install-dependencies/HDF5/lib/")
  find_library(HDF5_lib "libhdf5.a" HINTS "${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/lib/")

endif()



 if(HDF5_lib AND HDF5_lib_debug)
   set(HDF5_LIBRARIES_hl optimized ${HDF5_hl_cpp_lib} debug ${HDF5_hl_cpp_lib_debug})
   set(HDF5_LIBRARIES_cpp optimized ${HDF5_cpp_lib} debug ${HDF5_cpp_lib_debug})
   set(HDF5_LIBRARIES optimized ${HDF5_lib} debug ${HDF5_lib_debug})
 elseif(HDF5_lib)
   set(HDF5_LIBRARIES_hl ${HDF5_hl_cpp_lib})
   set(HDF5_LIBRARIES_cpp ${HDF5_cpp_lib})
   set(HDF5_LIBRARIES ${HDF5_lib})
 elseif(HDF5_lib_debug)
   set(HDF5_LIBRARIES_hl ${HDF5_hl_cpp_lib_debug})
   set(HDF5_LIBRARIES_cpp ${HDF5_cpp_lib_debug})
   set(HDF5_LIBRARIES ${HDF5_lib_debug})
 else()
   message(SEND_ERROR "No HDF5_lib found!")
 endif()

target_link_libraries(${PLUGIN_NAME} ${HDF5_LIBRARIES_hl})
target_link_libraries(${PLUGIN_NAME} ${HDF5_LIBRARIES_cpp})
target_link_libraries(${PLUGIN_NAME} ${HDF5_LIBRARIES})

set_target_properties(
    ${PLUGIN_NAME} PROPERTIES
    DEFINE_SYMBOL OSIMPLUGIN_EXPORTS
    PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

install(TARGETS ${PLUGIN_NAME} DESTINATION plugin)


add_subdirectory(dev_tools)
add_subdirectory(cmd_tools)
