set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(Dust_InfantryRobotChassis C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(DustOptions)

add_library(dust_options INTERFACE)
dust_configure_options(dust_options)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Board/dm-h723/STM32H723VGTX_FLASH.ld)

add_executable(${PROJECT_NAME}.elf)
target_sources(${PROJECT_NAME}.elf PRIVATE ${LINKER_SCRIPT})
target_link_libraries(${PROJECT_NAME}.elf PRIVATE dust_options)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
    -mcpu=cortex-m7 -mthumb -mthumb-interwork
    -mfloat-abi=hard -mfpu=fpv4-sp-d16
    -T ${LINKER_SCRIPT}
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_options(${PROJECT_NAME}.elf PRIVATE -Wl,--no-warn-rwx-segments)
endif()

target_link_directories(${PROJECT_NAME}.elf PRIVATE ${CMAKE_SOURCE_DIR}/ThirdParty/ARM/DSP/Lib)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE libarm_cortexM7lfsp_math.a)

add_subdirectory(ThirdParty)
add_subdirectory(Board/dm-h723)
add_subdirectory(Platform)
add_subdirectory(System)
add_subdirectory(Algorithm)
add_subdirectory(Drivers)
add_subdirectory(Communication)
add_subdirectory(Device)
add_subdirectory(App)
add_subdirectory(daemon_supervisor)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    dust_thirdparty
    dust_board_dm_h723
    dust_platform
    dust_system
    dust_algorithm
    dust_drivers
    dust_communication
    dust_device
    dust_interaction
    dust_daemon_supervisor
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} Building ${BIN_FILE}")
