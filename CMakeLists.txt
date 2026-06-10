# ============================================================
# STM32H723 GCC CMake 模板 — 自动生成，无需手动维护 include
# ============================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 4.0)

# ---- 交叉编译工具链 ----
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---- 项目设置 ----
project(hero_rtos C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# ---- 芯片与 FPU 配置 (STM32H723 Cortex-M7 单精度) ----
add_compile_definitions(
        ARM_MATH_CM7
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
)
add_compile_options(
        -mcpu=cortex-m7
        -mthumb
        -mthumb-interwork
        -mfloat-abi=hard
        -mfpu=fpv5-sp-d16
)
add_link_options(
        -mcpu=cortex-m7
        -mthumb
        -mthumb-interwork
        -mfloat-abi=hard
        -mfpu=fpv5-sp-d16
)

add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# 汇编预处理
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# ---- 优化等级 ----
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

# ============================================================
# 自动发现 include 目录
# 只要头文件在 Core/ Drivers/ Middlewares/ USER/ 下，自动加入
# 新增模块无需修改此文件
# ============================================================
set(INCLUDE_ROOTS
        Core/Inc
        Drivers
        Middlewares
        USB_DEVICE
        USER
)
foreach(root ${INCLUDE_ROOTS})
    file(GLOB_RECURSE _headers LIST_DIRECTORIES false ${root}/*.h ${root}/*.hpp)
    foreach(h ${_headers})
        get_filename_component(_dir ${h} DIRECTORY)
        list(APPEND AUTO_INCLUDE_DIRS ${_dir})
    endforeach()
endforeach()
list(REMOVE_DUPLICATES AUTO_INCLUDE_DIRS)
include_directories(${AUTO_INCLUDE_DIRS})

# ============================================================
# 自动收集源文件
# ============================================================
file(GLOB_RECURSE SOURCES
        "Core/*.*"
        "Drivers/*.*"
        "Middlewares/*.*"
        "USB_DEVICE/*.*"
        "USER/*.*"
)

# ---- 宏定义 ----
add_definitions(-DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx)

# ---- 链接脚本 ----
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H723VGTX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-T ${LINKER_SCRIPT})

# ---- 生成 ELF ----
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# ---- 链接 CMSIS-DSP 库 (GCC, Cortex-M7, 单精度) ----
target_link_directories(${PROJECT_NAME}.elf PRIVATE ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE arm_cortexM7lfsp_math)

# ---- 生成 HEX / BIN ----
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
)
