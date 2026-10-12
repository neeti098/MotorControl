# GNU Arm bare-metal toolchain file.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy)
# Defined even though the scaffold does not use them. An undefined variable in a
# POST_BUILD `COMMAND ${VAR} file.elf` expands to nothing, so ninja runs `file.elf`
# as the command -- on Windows that raises the shell's "Select an app to open this
# .elf file" dialog after EVERY build, and the build otherwise looks fine. Measured
# on two testers' machines after an agent added `COMMAND ${CMAKE_SIZE} ...`, which is
# the standard bare-metal idiom and the one variable the scaffold had left unset.
set(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size)
set(CMAKE_OBJDUMP      ${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_NM           ${TOOLCHAIN_PREFIX}nm)
set(CMAKE_AR           ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_RANLIB       ${TOOLCHAIN_PREFIX}ranlib)
set(CMAKE_STRIP        ${TOOLCHAIN_PREFIX}strip)
# Don't try to LINK during the compiler check (no startup/linker yet at configure time).
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections")
