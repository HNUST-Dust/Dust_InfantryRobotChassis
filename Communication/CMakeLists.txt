# Communication

file(GLOB_RECURSE COMMUNICATION_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

list(LENGTH COMMUNICATION_SOURCES COMMUNICATION_SOURCES_COUNT)
if(COMMUNICATION_SOURCES_COUNT EQUAL 0)
    add_library(dust_communication INTERFACE)
    target_link_libraries(dust_communication INTERFACE dust_options)
else()
    add_library(dust_communication OBJECT)
    target_sources(dust_communication PRIVATE ${COMMUNICATION_SOURCES})
    target_link_libraries(dust_communication PRIVATE dust_options)
endif()
