cmake_minimum_required (VERSION 2.8.11)
project(IMUA)

SET(LIBNAME "IMUAnalysis")

include_directories(include)

list(APPEND sources src/generic.cpp)



ADD_LIBRARY(${LIBNAME} STATIC
    ${sources}
)

INSTALL(TARGETS ${LIBNAME} DESTINATION lib)

INSTALL(FILES
    include/generic.h
    DESTINATION include/${LIBNAME}
)



add_executable(hello main.cpp)
target_link_libraries(hello ${LIBNAME})