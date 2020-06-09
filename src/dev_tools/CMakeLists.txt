#Create Tools library
set(JAM_TOOLS_NAME "jam_tools"
CACHE STRING "Name of static library to create")

#Specify Files
file(GLOB SOURCE_FILES */*.cpp */*.c)
file(GLOB INCLUDE_FILES */*.h)

#Specify Plugin Details
add_library(${JAM_TOOLS_NAME} STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
target_link_libraries(${JAM_TOOLS_NAME} ${OpenSim_LIBRARIES})
set_target_properties(${JAM_TOOLS_NAME} PROPERTIES DEBUG_POSTFIX "_d")
SET_TARGET_PROPERTIES(${JAM_TOOLS_NAME} PROPERTIES FOLDER jam_tools)
set_property(TARGET ${JAM_TOOLS_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
install(TARGETS ${JAM_TOOLS_NAME} DESTINATION jam_tools)
