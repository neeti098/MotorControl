# Select the vendor-supplied FreeRTOS kernel sources. The Cube package version and
# upstream license are preserved in the project-local user/drivers tree.
set(FREERTOS_ROOT ${CMAKE_SOURCE_DIR}/user/drivers/freertos_kernel/Source)
target_sources(${PROJECT_NAME}.elf PRIVATE
    ${FREERTOS_ROOT}/tasks.c
    ${FREERTOS_ROOT}/queue.c
    ${FREERTOS_ROOT}/list.c
    ${FREERTOS_ROOT}/portable/MemMang/heap_4.c
    ${FREERTOS_ROOT}/portable/GCC/ARM_CM4F/port.c)
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${FREERTOS_ROOT}/include
    ${FREERTOS_ROOT}/portable/GCC/ARM_CM4F)
