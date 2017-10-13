#set(EXENAME example)
add_library(gpmf_parser SHARED gpmf_parser.cpp)
#install(TARGETS ${EXENAME} DESTINATION lib)

install(TARGETS gpmf_parser DESTINATION lib)
