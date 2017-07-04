set(EXENAME example)
add_executable(${EXENAME} example.cpp gpmf_parser.c)
target_link_libraries(${EXENAME} ${LIBNAME})
install(TARGETS ${EXENAME} DESTINATION bin)
