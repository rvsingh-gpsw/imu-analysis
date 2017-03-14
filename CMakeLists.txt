# Project name
project(IMUA)

# CMake minimum version
cmake_minimum_required (VERSION 2.8.11)

# Where to look for headers
include_directories(include)

# Create the list of source files
set(SOURCES src/types.cpp
            src/generic.cpp
            src/snowboard.cpp
            src/surfing.cpp)

# Create a library
set(LIBNAME IMUAnalysis)
add_library(${LIBNAME} STATIC ${SOURCES})

# Create the list of header files
set(HEADERS include/types.h
            include/generic.h
            include/snowboard.h
            include/surfing.h)

# Create an executable using the created library
set(EXENAME detector)
add_executable(${EXENAME} main.cpp gpmf_parser.c)
target_link_libraries(${EXENAME} ${LIBNAME})

# Indicate where to install the library, the executable and the headers
install(TARGETS ${LIBNAME} DESTINATION lib)
install(TARGETS ${EXENAME} DESTINATION bin)
install(FILES   ${HEADERS} DESTINATION include/${LIBNAME})





#message(STATUS "Coucou : ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
#message(STATUS "Coucou")
#set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
#set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_DIR}/lib )
#set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

#INSTALL(FILES
#    include/generic.h
#    DESTINATION include/${LIBNAME}
#)
