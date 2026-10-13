cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32f103_CubemxCLI_demo0)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ---------------------------------------------------------------------------
# 4G 模组驱动选择：自动跟随 Core/Inc/board_config.h 里的模块宏
#   MODULE_FSC_AT  → 飞思创 AT 指令固件   Core/Src/dtu_ctrl.c
#   MODULE_YINERDA → 银尔达透传固件(默认) Core/Src/dtu.c
# 两者对外 API 一致，切换宏后 build 会自动重新 configure 并改编译对应 .c。
# ---------------------------------------------------------------------------
set(_BOARD_CFG "${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/board_config.h")
file(STRINGS "${_BOARD_CFG}" _MODULE_FSC_AT_LINE
     REGEX "^[ \t]*#[ \t]*define[ \t]+MODULE_FSC_AT")
if(_MODULE_FSC_AT_LINE)
    set(DTU_DRIVER_SRC Core/Src/dtu_ctrl.c)
    message(STATUS "4G module driver: Fibocom AT command (MODULE_FSC_AT)")
else()
    set(DTU_DRIVER_SRC Core/Src/dtu.c)
    message(STATUS "4G module driver: Yinerda transparent (MODULE_YINERDA)")
endif()
# 让 CMake 监视 board_config.h：改动模组宏后再次 build 会自动重新 configure
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_BOARD_CFG}")

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/sc7a20h.c
    Core/Src/iccid_storage.c
    ${DTU_DRIVER_SRC}
    Core/Src/board_io.c
    Core/Src/rtc_app.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
