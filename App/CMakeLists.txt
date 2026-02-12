# Interaction

file(GLOB_RECURSE INTERACTION_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

list(LENGTH INTERACTION_SOURCES INTERACTION_SOURCES_COUNT)
if(INTERACTION_SOURCES_COUNT EQUAL 0)
    add_library(dust_interaction INTERFACE)
    target_link_libraries(dust_interaction INTERFACE dust_options)
else()
    add_library(dust_interaction OBJECT)
    target_sources(dust_interaction PRIVATE ${INTERACTION_SOURCES})
    target_link_libraries(dust_interaction PRIVATE dust_options)
endif()
