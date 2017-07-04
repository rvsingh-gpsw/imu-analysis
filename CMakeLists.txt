# Project name
project(IMUANALYSIS)

# CMake minimum version
cmake_minimum_required (VERSION 2.8.11)

# Where to look for headers
include_directories(include)

# Create the list of source files
set(SOURCES src/types.cpp
            src/tools.cpp
            src/gp5imu_Madgwick.cpp
            src/generic.cpp
            src/snowboarding.cpp
            src/skateboarding.cpp
            src/mountainbiking.cpp
            src/surfing.cpp)

# Create a library
set(LIBNAME IMUAnalysis)
add_library(${LIBNAME} ${SOURCES})

# Create the list of header files
set(HEADERS include/types.h
            include/gp5imu_Madgwick.h
            include/generic.h
            include/snowboarding.h
            include/skateboarding.h
            include/mountainbiking.h
            include/surfing.h)

# Indicate where to install the library and the headers
install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES   ${HEADERS} DESTINATION include/${LIBNAME})

# By default, the executable is not compiled
set(COMPILE_EXECUTABLE OFF CACHE BOOL "Compile executable")
if (COMPILE_EXECUTABLE)
    add_subdirectory(example)
    add_subdirectory(tests)
endif()