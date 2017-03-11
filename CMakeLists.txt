# Project name
project(IMUA)

cmake_minimum_required (VERSION 2.8.11)


# Lib name
set(LIBNAME IMUAnalysis)

# Where to look for headers
include_directories(include)

# Create the list of source files
set(SRC src/generic.cpp
        src/snowboard.cpp
        src/types.cpp)

# Create a library
add_library(${LIBNAME} STATIC ${SRC})

# Indicate where to install the library
#install(TARGETS ${LIBNAME} DESTINATION lib)

#INSTALL(FILES
#    include/generic.h
#    DESTINATION include/${LIBNAME}
#)

# Create an executable using the created library
add_executable(hello main.cpp)
target_link_libraries(hello ${LIBNAME})
