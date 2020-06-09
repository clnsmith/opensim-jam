#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SimTKcommon" for configuration "Release"
set_property(TARGET SimTKcommon APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(SimTKcommon PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/lib64/libblas.so;/usr/lib64/liblapack.so;/usr/lib64/libblas.so;pthread;rt;dl;m"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib64/libSimTKcommon.so.3.7"
  IMPORTED_SONAME_RELEASE "libSimTKcommon.so.3.7"
  )

list(APPEND _IMPORT_CHECK_TARGETS SimTKcommon )
list(APPEND _IMPORT_CHECK_FILES_FOR_SimTKcommon "${_IMPORT_PREFIX}/lib64/libSimTKcommon.so.3.7" )

# Import target "SimTKmath" for configuration "Release"
set_property(TARGET SimTKmath APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(SimTKmath PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "SimTKcommon;/usr/lib64/libblas.so;/usr/lib64/liblapack.so;/usr/lib64/libblas.so;pthread;rt;dl;m"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib64/libSimTKmath.so.3.7"
  IMPORTED_SONAME_RELEASE "libSimTKmath.so.3.7"
  )

list(APPEND _IMPORT_CHECK_TARGETS SimTKmath )
list(APPEND _IMPORT_CHECK_FILES_FOR_SimTKmath "${_IMPORT_PREFIX}/lib64/libSimTKmath.so.3.7" )

# Import target "SimTKsimbody" for configuration "Release"
set_property(TARGET SimTKsimbody APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(SimTKsimbody PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "SimTKmath;SimTKcommon;/usr/lib64/libblas.so;/usr/lib64/liblapack.so;/usr/lib64/libblas.so;pthread;rt;dl;m"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib64/libSimTKsimbody.so.3.7"
  IMPORTED_SONAME_RELEASE "libSimTKsimbody.so.3.7"
  )

list(APPEND _IMPORT_CHECK_TARGETS SimTKsimbody )
list(APPEND _IMPORT_CHECK_FILES_FOR_SimTKsimbody "${_IMPORT_PREFIX}/lib64/libSimTKsimbody.so.3.7" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
