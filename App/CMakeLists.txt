# Interaction

file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)


list(LENGTH APP_SOURCES APP_SOURCES_COUNT)
if(APP_SOURCES_COUNT EQUAL 0)
    add_library(dust_app INTERFACE)
    target_link_libraries(dust_app INTERFACE dust_options)
else()
    add_library(dust_app OBJECT)
    target_sources(dust_app PRIVATE ${APP_SOURCES})
    target_link_libraries(dust_app PRIVATE dust_options)
endif()
