set(TESTNAME tests)
add_executable(${TESTNAME} tests.cpp)
target_link_libraries(${TESTNAME} ${LIBNAME})
install(TARGETS ${TESTNAME} DESTINATION bin)