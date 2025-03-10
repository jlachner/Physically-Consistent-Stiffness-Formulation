cmake_minimum_required(VERSION 3.2)

project(ChristoffelSymbols)

# Find Boost libraries
find_package(Boost REQUIRED COMPONENTS filesystem system thread date_time chrono)

# Include directories
set(includeDir
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Explicit-cpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/myFRIClient/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Explicit-cpp/Eigen
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/FTSensor/include
)
include_directories(${includeDir})

# Make all source files visible in the IDE
set(sourceFiles "")
file(GLOB_RECURSE sourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# Add executable
add_executable(${PROJECT_NAME} ${sourceFiles} ${headerFiles})


# Linking
set(EXPLICIT_CPP_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Explicit-cpp/lib/libExplicit-cpp.so" CACHE STRING "Explicit-cpp library")
set(FRI_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/myFRIClient/lib/libFRIClient.a" CACHE STRING "FRI library")
set(SENSOR_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/FTSensor/lib/libFTSensor.so" CACHE STRING "FTSensor library")

set(linkLibraries
${EXPLICIT_CPP_LIBRARY}
${FRI_LIBRARY}
${SENSOR_LIBRARY}
${Boost_LIBRARIES}
)

# Some parts of the code go into an additional library
target_link_libraries(${PROJECT_NAME} ${linkLibraries})

# The output directory
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../build/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../build/Release"
)

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h"
)

# Install library
set(libOutputDir ${CMAKE_INSTALL_PREFIX}/lib)
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${libOutputDir}
        ARCHIVE DESTINATION ${libOutputDir}
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/application
)
