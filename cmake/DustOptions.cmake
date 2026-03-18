# Centralized compile/link options for the Dust project.
# Kept as a separate file so subdirectories can stay minimal.

function(dust_configure_options options_target)
    # Language standards (propagate to consumers)
    target_compile_features(${options_target} INTERFACE c_std_11 cxx_std_17)

    target_compile_definitions(${options_target} INTERFACE
        ARM_MATH_CM4
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
        DEBUG
        USE_PWR_LDO_SUPPLY
        USE_HAL_DRIVER
        STM32H723xx
    )

    target_compile_options(${options_target} INTERFACE
        -mcpu=cortex-m7 -mthumb -mthumb-interwork
        -mfloat-abi=hard -mfpu=fpv4-sp-d16
        -ffunction-sections -fdata-sections -fno-common -fmessage-length=0
        $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
        $<$<CONFIG:Release>:-Ofast>
        $<$<CONFIG:RelWithDebInfo>:-Ofast -g>
        $<$<CONFIG:MinSizeRel>:-Os>
        $<$<OR:$<CONFIG:Debug>,$<STREQUAL:${CMAKE_BUILD_TYPE},>>:-Og -g>
    )

    target_include_directories(${options_target} INTERFACE
        ${CMAKE_SOURCE_DIR}/Board/dm-h723
        ${CMAKE_SOURCE_DIR}/Board/dm-h723/Inc
        ${CMAKE_SOURCE_DIR}/ThirdParty/STM32H7xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/ThirdParty/STM32H7xx_HAL_Driver/Inc/Legacy
        ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/include
        ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/CMSIS_RTOS_V2
        ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F
        ${CMAKE_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Core/Inc
        ${CMAKE_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Class/CDC/Inc
        ${CMAKE_SOURCE_DIR}/ThirdParty/ARM/DSP/Inc
        ${CMAKE_SOURCE_DIR}/ThirdParty/CMSIS/Device/ST/STM32H7xx/Include
        ${CMAKE_SOURCE_DIR}/ThirdParty/CMSIS/Include

        ${CMAKE_SOURCE_DIR}/App
        ${CMAKE_SOURCE_DIR}/Algorithm
        ${CMAKE_SOURCE_DIR}/Device
        ${CMAKE_SOURCE_DIR}/Platform
        ${CMAKE_SOURCE_DIR}/daemon_supervisor
        ${CMAKE_SOURCE_DIR}/Communication
        ${CMAKE_SOURCE_DIR}/Drivers
        ${CMAKE_SOURCE_DIR}/ThirdParty/SEGGER/Config
        ${CMAKE_SOURCE_DIR}/ThirdParty/SEGGER/RTT
    )
endfunction()
