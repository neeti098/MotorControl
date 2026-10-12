cmake_minimum_required(VERSION 3.22)
# Toolchain must be set before project(): -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
project(sensorless_pmsm_foc C ASM)
set(CMAKE_C_STANDARD 11)

# CPU flags for STM32G431xx (Cortex core from the STM32 family). Applied to BOTH compile and link.
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
separate_arguments(CPU_FLAGS UNIX_COMMAND "${CPU_FLAGS}")

add_compile_options(${CPU_FLAGS} -ffunction-sections -fdata-sections -Wall)
# USE_HAL_DRIVER selects the HAL; STM32G431xx picks the exact device header in CMSIS.
add_compile_definitions(USE_HAL_DRIVER STM32G431xx)

# HAL/core/app sources. The entry point is Core/Src/main.c — do NOT glob src/*.c: a leftover
# src/main.c from an older scaffold would define a second main() and break the link.
# *_template.c files were excluded at copy time (they duplicate symbols).
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Core/Src/*.c
    ${CMAKE_SOURCE_DIR}/firmware/*.c
    ${CMAKE_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/*.c)
# Vendor sources registered as user components are attached via their component manifest below;
# exclude them from this application glob to avoid compiling duplicate kernel copies.
list(FILTER PROJECT_SOURCES EXCLUDE REGEX "/user/drivers/")
# firmware/{tests,docs,scripts}/ hold host test code, documentation and helpers -- never
# firmware sources. Generated unit-test bundles land in firmware/docs/unit_tests/, and the
# globs above use CONFIGURE_DEPENDS, so without this they enter the image on the NEXT build.
list(FILTER PROJECT_SOURCES EXCLUDE REGEX "/firmware/(tests|docs|scripts)/")

# Every folder under firmware/ that holds a header goes on the include path, so headers resolve by
# bare filename from any layer and a NEW layer folder needs no edit here. Listing the layers by
# hand is what broke `#include "board.h"` when bsp/ was added.
file(GLOB_RECURSE FW_HEADERS CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/firmware/*.h)
list(FILTER FW_HEADERS EXCLUDE REGEX "/firmware/(tests|docs|scripts)/")
set(FW_INCLUDE_DIRS "")
foreach(_hdr ${FW_HEADERS})
    get_filename_component(_dir "${_hdr}" DIRECTORY)
    list(APPEND FW_INCLUDE_DIRS "${_dir}")
endforeach()
if(FW_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES FW_INCLUDE_DIRS)
endif()

set(STARTUP ${CMAKE_SOURCE_DIR}/Core/Startup/startup_stm32g431rbtx.s)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32G431RBTX_FLASH.ld)

add_executable(${PROJECT_NAME}.elf ${PROJECT_SOURCES} ${STARTUP})

# FreeRTOS kernel version supplied by the installed STM32CubeG4 package.
set(FREERTOS_ROOT ${CMAKE_SOURCE_DIR}/user/drivers/freertos_kernel/Source)
target_sources(${PROJECT_NAME}.elf PRIVATE
    ${FREERTOS_ROOT}/tasks.c
    ${FREERTOS_ROOT}/queue.c
    ${FREERTOS_ROOT}/list.c
    ${FREERTOS_ROOT}/portable/GCC/ARM_CM4F/port.c)
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${FREERTOS_ROOT}/include
    ${FREERTOS_ROOT}/portable/GCC/ARM_CM4F)

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${FW_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${CPU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map
    --specs=nano.specs --specs=nosys.specs
    -lc -lm)

# .hex/.bin next to the .elf for flashing.
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin)


# >>> FIRMGEN_USER_COMPONENTS
# User-provided components (custom linker script, existing libraries/drivers).
# Generated from firmgen_user_components.json; see .craftif/user_components.cmake.
include("${CMAKE_CURRENT_LIST_DIR}/.craftif/user_components.cmake" OPTIONAL)
if(COMMAND firmgen_attach_user_components)
  firmgen_attach_user_components(${PROJECT_NAME}.elf)
endif()
# <<< FIRMGEN_USER_COMPONENTS
