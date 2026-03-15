# Board (dm-h723)

file(GLOB_RECURSE BOARD_SOURCES CONFIGURE_DEPENDS
    "Src/*.c" "Src/*.cpp" "Src/*.s" "Src/*.S"
    "startup_stm32h723vgtx.s" "startup_stm32h723vgtx.S"
    "Port/*.c" "Port/*.cpp" "Port/*.s" "Port/*.S"
)

list(LENGTH BOARD_SOURCES BOARD_SOURCES_COUNT)
if(BOARD_SOURCES_COUNT EQUAL 0)
    add_library(dust_board_dm_h723 INTERFACE)
    target_link_libraries(dust_board_dm_h723 INTERFACE dust_options)
else()
    add_library(dust_board_dm_h723 OBJECT)
    target_sources(dust_board_dm_h723 PRIVATE ${BOARD_SOURCES})
    target_link_libraries(dust_board_dm_h723 PRIVATE dust_options)
endif()
