# Device

file(GLOB_RECURSE DEVICE_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

list(LENGTH DEVICE_SOURCES DEVICE_SOURCES_COUNT)
if(DEVICE_SOURCES_COUNT EQUAL 0)
    add_library(dust_device INTERFACE)
    target_link_libraries(dust_device INTERFACE dust_options)
else()
    add_library(dust_device OBJECT)
    target_sources(dust_device PRIVATE ${DEVICE_SOURCES})
    target_link_libraries(dust_device PRIVATE dust_options)
endif()
