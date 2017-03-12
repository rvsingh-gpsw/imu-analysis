# Project name
project(IMUA)

# CMake minimum version
cmake_minimum_required (VERSION 2.8.11)

# Where to look for headers
include_directories(include)

# Create the list of source files
set(SRC src/generic.cpp
        src/snowboard.cpp
        src/types.cpp)

# Create a library
set(LIBNAME IMUAnalysis)
add_library(${LIBNAME} STATIC ${SRC})

# Indicate where to install the library
#install(TARGETS ${LIBNAME} DESTINATION lib)

#INSTALL(FILES
#    include/generic.h
#    DESTINATION include/${LIBNAME}
#)

# Create an executable using the created library
set(EXENAME detector)
add_executable(${EXENAME} main.cpp gpmf_parser.c)
target_link_libraries(${EXENAME} ${LIBNAME})
