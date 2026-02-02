# Platform

file(GLOB_RECURSE PLATFORM_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

list(LENGTH PLATFORM_SOURCES PLATFORM_SOURCES_COUNT)
if(PLATFORM_SOURCES_COUNT EQUAL 0)
    add_library(dust_platform INTERFACE)
    target_link_libraries(dust_platform INTERFACE dust_options)
else()
    add_library(dust_platform OBJECT)
    target_sources(dust_platform PRIVATE ${PLATFORM_SOURCES})
    target_link_libraries(dust_platform PRIVATE dust_options)
endif()
