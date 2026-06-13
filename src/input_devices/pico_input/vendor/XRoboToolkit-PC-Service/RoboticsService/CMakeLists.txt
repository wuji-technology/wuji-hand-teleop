# Generated from RoboticsService.pro.

cmake_minimum_required(VERSION 3.16)
option(AUTO_BUILD_OVERSEA "Command Line Argument" OFF)
option(NOT_LOCAL_BUILD "Command Line Argument" OFF)
option(BUILD_LIB_PATH "Command Line Argument" "")
option(AUTO_BUILD_NUM "Command Line Argument" "")


project(RoboticsServiceProject)

cmake_host_system_information(RESULT ISA_NAME QUERY OS_PLATFORM)
message("OS_PLATFORM: ${ISA_NAME}")


if(AUTO_BUILD_NUM)
    set(BUILD_VERION_NUM 1.0.0.${AUTO_BUILD_NUM})
else()
    set(BUILD_VERION_NUM 1.0.0.0)
endif()


if(BUILD_LIB_PATH)
    message("use input path:${BUILD_LIB_PATH}")
    set(BUILD_LIB_PATH_VALUE ${BUILD_LIB_PATH})
else()
    set(BUILD_LIB_PATH_VALUE ${CMAKE_PREFIX_PATH})
endif()

if(AUTO_BUILD_OVERSEA)
    message("build overseas app.")
    add_definitions(-DOVERSEAS)
endif()


set(INSTALL_DIR  ${PROJECT_SOURCE_DIR}/bin)
message(STATUS ${INSTALL_DIR})

if(NOT EXISTS ${INSTALL_DIR})
    file(MAKE_DIRECTORY ${INSTALL_DIR})
endif()


find_package(Qt6 REQUIRED COMPONENTS Core5Compat)
find_package(QT NAMES Qt5 Qt6 REQUIRED COMPONENTS Core)


if(ISA_NAME STREQUAL "aarch64")
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Network)
else()
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Network)
endif()

if(WIN32)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

    add_subdirectory(CommonUtils)
    add_subdirectory(DeviceConnectionManager)
    add_subdirectory(Business)
    add_subdirectory(PXREAGRPCServer)
    add_subdirectory(RoboticsServiceProcess)

    add_dependencies(DeviceConnectionManager CommonUtils)
    add_dependencies(Business CommonUtils DeviceConnectionManager)
    add_dependencies(PXREAGRPCServer DeviceConnectionManager CommonUtils Business)
    add_dependencies(RoboticsServiceProcess PXREAGRPCServer  Business CommonUtils DeviceConnectionManager)
endif()


if(UNIX)
    add_subdirectory(CommonUtils)
    add_subdirectory(DeviceConnectionManager)
    add_subdirectory(Business)
    add_subdirectory(PXREAGRPCServer)
    add_subdirectory(RoboticsServiceProcess)

    add_dependencies(DeviceConnectionManager CommonUtils)
    add_dependencies(Business CommonUtils DeviceConnectionManager)
    add_dependencies(PXREAGRPCServer DeviceConnectionManager CommonUtils Business)
    add_dependencies(RoboticsServiceProcess PXREAGRPCServer  Business CommonUtils DeviceConnectionManager)
endif()
