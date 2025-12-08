# 添加板卡和固件选择选项
set(BOARD "dm-h723" CACHE STRING "Target board")
set(FIRMWARE "DownBoard_Interaction" CACHE STRING "Target firmware")

# 包含板卡特定配置
include(Board/${BOARD}/board.cmake)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)

# 明确指定生成器和工具链
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(FW C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 软件浮点选项（如果需要）
#add_compile_options(-mfloat-abi=soft)

# 通用编译选项 (无关芯片架构)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# 构建类型优化选项
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(
        Board/${BOARD}/
        Board/${BOARD}/Inc 
        ThirdParty/STM32H7xx_HAL_Driver/Inc 
        ThirdParty/STM32H7xx_HAL_Driver/Inc/Legacy 
        ThirdParty/FreeRTOS/Source/include 
        ThirdParty/FreeRTOS/Source/CMSIS_RTOS_V2 
        ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F 
        ThirdParty/STM32_USB_Device_Library/Core/Inc 
        ThirdParty/STM32_USB_Device_Library/Class/CDC/Inc 
        ThirdParty/ARM/DSP/Inc
        ThirdParty/CMSIS/Device/ST/STM32H7xx/Include 
        ThirdParty/CMSIS/Include 
        ThirdParty/SEGGER/Config
        ThirdParty/SEGGER/RTT
)

include_directories(
        Firmware/${FIRMWARE}/
        Common/Algorithm/
        Common/Device/
        Common/Communication/
        Common/Drivers/
)

# 这些定义现在在board.cmake中处理

file(
    GLOB_RECURSE SOURCES 
    "Board/${BOARD}/*.*" 
    "ThirdParty/*.*" 
    "Firmware/${FIRMWARE}/*.*" 
    "Common/Algorithm/*.*" 
    "Common/Communication/*.*"
    "Common/Drivers/*.*" 
    "Common/Device/*.*"
)

# 链接脚本路径由board.cmake提供

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
# 链接选项中的芯片架构选项已移到board.cmake
add_link_options(-T ${LINKER_SCRIPT})

link_directories(${CMAKE_SOURCE_DIR}/ThirdParty/ARM/DSP/Lib)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${MATH_LIB})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

# 创建固件保留目录
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/firmware_releases)

# 构建后步骤：生成固件文件并复制到保留目录
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${BIN_FILE}
        COMMAND ${CMAKE_COMMAND} -E copy ${BIN_FILE} ${CMAKE_SOURCE_DIR}/firmware_releases/${BOARD}_${FIRMWARE}.bin
        COMMAND ${CMAKE_COMMAND} -E copy ${HEX_FILE} ${CMAKE_SOURCE_DIR}/firmware_releases/${BOARD}_${FIRMWARE}.hex
        COMMENT "Building firmware and saving to releases")
