
Project(JAM_OpenSim)
cmake_minimum_required(VERSION 3.3)

set(JAM_OPENSIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE
        PATH "Top-level directory of jam_opensim repo")

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
      SET(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../install" CACHE PATH "Location to install plugin libraries" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#Specify OpenSim Stuff
#===============================================================================
#set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
#    CACHE PATH "Top-level directory of OpenSim install")

#set(OPENSIM_INSTALL_DIR ""
#    CACHE PATH "Top-level directory of OpenSim install")

#set(CMAKE_PREFIX_PATH "${OPENSIM_INSTALL_DIR}" CACHE PATH "Add OpenSim install directory" FORCE)

# OpenSim uses C++11 language features.
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    if(APPLE)
        if(XCODE)
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
            endif()
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()

find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

include_directories(${OpenSim_INCLUDE_DIRS})
link_directories(${OpenSim_LIB_DIR})


# Find external Libraries
#===============================================================================
FILE(GLOB EXTERNAL_LIB_NAMES RELATIVE "${JAM_OPENSIM_DIR}/dependencies" "${JAM_OPENSIM_DIR}/dependencies/*")
FOREACH(external_lib_name ${EXTERNAL_LIB_NAMES})
  include_directories(${JAM_OPENSIM_DIR}/dependencies/${external_lib_name}/)
ENDFOREACH(external_lib_name)

include_directories(${JAM_OPENSIM_DIR}/../install-dependencies/HDF5/include/)

#Add Subdirectories
#===============================================================================

include_directories("${JAM_OPENSIM_DIR}/src")

FILE(GLOB TOOL_NAMES RELATIVE "${CMAKE_CURRENT_LIST_DIR}/src/dev_tools" "${CMAKE_CURRENT_LIST_DIR}/src/dev_tools/*")
FOREACH(tool_name ${TOOL_NAMES})
    IF(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/src/dev_tools/${tool_name}")
        include_directories("${JAM_OPENSIM_DIR}/src/dev_tools/${tool_name}")
    ENDIF()
ENDFOREACH(tool_name)

link_directories("${CMAKE_INSTALL_PREFIX}/src/dev_tools")
link_directories("${CMAKE_INSTALL_PREFIX}/src")

add_subdirectory(src)

# Setup Doxygen
#==============
# first we can indicate the documentation build as an option and set it to ON by default
option(BUILD_DOC "Build documentation" ON)

# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/documentation/doxygen/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen needs to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)
