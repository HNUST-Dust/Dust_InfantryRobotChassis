# System

file(GLOB_RECURSE SYSTEM_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

list(LENGTH SYSTEM_SOURCES SYSTEM_SOURCES_COUNT)
if(SYSTEM_SOURCES_COUNT EQUAL 0)
    add_library(dust_system INTERFACE)
    target_link_libraries(dust_system INTERFACE dust_options)
else()
    add_library(dust_system OBJECT)
    target_sources(dust_system PRIVATE ${SYSTEM_SOURCES})
    target_link_libraries(dust_system PRIVATE dust_options)
endif()
