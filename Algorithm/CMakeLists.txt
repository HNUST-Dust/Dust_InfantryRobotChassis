# Algorithm

file(GLOB_RECURSE ALGORITHM_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

list(LENGTH ALGORITHM_SOURCES ALGORITHM_SOURCES_COUNT)
if(ALGORITHM_SOURCES_COUNT EQUAL 0)
    add_library(dust_algorithm INTERFACE)
    target_link_libraries(dust_algorithm INTERFACE dust_options)
else()
    add_library(dust_algorithm OBJECT)
    target_sources(dust_algorithm PRIVATE ${ALGORITHM_SOURCES})
    target_link_libraries(dust_algorithm PRIVATE dust_options)
endif()
