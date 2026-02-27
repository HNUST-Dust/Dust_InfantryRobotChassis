# ThirdParty
# Keep this list focused on actual compiled sources.
# Header-only dependencies (e.g., Eigen) don't need to be compiled.

file(GLOB_RECURSE THIRD_PARTY_SOURCES CONFIGURE_DEPENDS
    "FreeRTOS/*.c" "FreeRTOS/*.cpp" "FreeRTOS/*.s" "FreeRTOS/*.S"
    "STM32H7xx_HAL_Driver/*.c" "STM32H7xx_HAL_Driver/*.cpp"
    "STM32_USB_Device_Library/*.c" "STM32_USB_Device_Library/*.cpp"
    "SEGGER/*.c" "SEGGER/*.cpp" "SEGGER/*.s" "SEGGER/*.S"
    "CMSIS/*.c" "CMSIS/*.cpp" "CMSIS/*.s" "CMSIS/*.S"
)

list(LENGTH THIRD_PARTY_SOURCES THIRD_PARTY_SOURCES_COUNT)
if(THIRD_PARTY_SOURCES_COUNT EQUAL 0)
    add_library(dust_thirdparty INTERFACE)
    target_link_libraries(dust_thirdparty INTERFACE dust_options)
else()
    add_library(dust_thirdparty OBJECT)
    target_sources(dust_thirdparty PRIVATE ${THIRD_PARTY_SOURCES})
    target_link_libraries(dust_thirdparty PRIVATE dust_options)
endif()
