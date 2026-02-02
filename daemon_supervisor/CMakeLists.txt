# daemon_supervisor

file(GLOB_RECURSE DAEMON_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

list(LENGTH DAEMON_SOURCES DAEMON_SOURCES_COUNT)
if(DAEMON_SOURCES_COUNT EQUAL 0)
    add_library(dust_daemon_supervisor INTERFACE)
    target_link_libraries(dust_daemon_supervisor INTERFACE dust_options)
else()
    add_library(dust_daemon_supervisor OBJECT)
    target_sources(dust_daemon_supervisor PRIVATE ${DAEMON_SOURCES})
    target_link_libraries(dust_daemon_supervisor PRIVATE dust_options)
endif()
