#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "osimLepton" for configuration "Release"
set_property(TARGET osimLepton APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimLepton PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/sdk/lib/libosimLepton.so"
  IMPORTED_SONAME_RELEASE "libosimLepton.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS osimLepton )
list(APPEND _IMPORT_CHECK_FILES_FOR_osimLepton "${_IMPORT_PREFIX}/sdk/lib/libosimLepton.so" )

# Import target "osimCommon" for configuration "Release"
set_property(TARGET osimCommon APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimCommon PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/sdk/lib/libosimCommon.so"
  IMPORTED_SONAME_RELEASE "libosimCommon.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS osimCommon )
list(APPEND _IMPORT_CHECK_FILES_FOR_osimCommon "${_IMPORT_PREFIX}/sdk/lib/libosimCommon.so" )

# Import target "osimSimulation" for configuration "Release"
set_property(TARGET osimSimulation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimSimulation PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/sdk/lib/libosimSimulation.so"
  IMPORTED_SONAME_RELEASE "libosimSimulation.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS osimSimulation )
list(APPEND _IMPORT_CHECK_FILES_FOR_osimSimulation "${_IMPORT_PREFIX}/sdk/lib/libosimSimulation.so" )

# Import target "osimActuators" for configuration "Release"
set_property(TARGET osimActuators APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimActuators PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/sdk/lib/libosimActuators.so"
  IMPORTED_SONAME_RELEASE "libosimActuators.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS osimActuators )
list(APPEND _IMPORT_CHECK_FILES_FOR_osimActuators "${_IMPORT_PREFIX}/sdk/lib/libosimActuators.so" )

# Import target "osimAnalyses" for configuration "Release"
set_property(TARGET osimAnalyses APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimAnalyses PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/sdk/lib/libosimAnalyses.so"
  IMPORTED_SONAME_RELEASE "libosimAnalyses.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS osimAnalyses )
list(APPEND _IMPORT_CHECK_FILES_FOR_osimAnalyses "${_IMPORT_PREFIX}/sdk/lib/libosimAnalyses.so" )

# Import target "osimTools" for configuration "Release"
set_property(TARGET osimTools APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimTools PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/sdk/lib/libosimTools.so"
  IMPORTED_SONAME_RELEASE "libosimTools.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS osimTools )
list(APPEND _IMPORT_CHECK_FILES_FOR_osimTools "${_IMPORT_PREFIX}/sdk/lib/libosimTools.so" )

# Import target "osimExampleComponents" for configuration "Release"
set_property(TARGET osimExampleComponents APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimExampleComponents PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/sdk/lib/libosimExampleComponents.so"
  IMPORTED_SONAME_RELEASE "libosimExampleComponents.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS osimExampleComponents )
list(APPEND _IMPORT_CHECK_FILES_FOR_osimExampleComponents "${_IMPORT_PREFIX}/sdk/lib/libosimExampleComponents.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
