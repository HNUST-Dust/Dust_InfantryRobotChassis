# Drivers

file(GLOB_RECURSE DRIVERS_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

list(LENGTH DRIVERS_SOURCES DRIVERS_SOURCES_COUNT)
if(DRIVERS_SOURCES_COUNT EQUAL 0)
    add_library(dust_drivers INTERFACE)
    target_link_libraries(dust_drivers INTERFACE dust_options)
else()
    add_library(dust_drivers OBJECT)
    target_sources(dust_drivers PRIVATE ${DRIVERS_SOURCES})
    target_link_libraries(dust_drivers PRIVATE dust_options)
endif()
