# Generated from CommonUtils.pro.

cmake_minimum_required(VERSION 3.16)
project(CommonUtils VERSION 1.0 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)

set(INSTALL_DIR  ${PROJECT_SOURCE_DIR}/../bin)

if(NOT EXISTS ${INSTALL_DIR})
    file(MAKE_DIRECTORY ${INSTALL_DIR})
endif()


set(CMAKE_CXX_STANDARD 11)


find_package(Qt6 REQUIRED COMPONENTS Core5Compat)

find_package(QT NAMES Qt5 Qt6 REQUIRED COMPONENTS Core Network)

qt_add_library(CommonUtils SHARED
    CommonUtils_global.h
    MessageHandler.h
    commonutils.cpp
    commonutils.h
)




target_include_directories(CommonUtils PUBLIC
    ../DeviceConnectionManager
)

target_compile_definitions(CommonUtils PUBLIC
    COMMONUTILS_LIBRARY
    QT_MESSAGELOGCONTEXT
)

target_link_libraries(CommonUtils PRIVATE Qt6::Core5Compat)

target_link_libraries(CommonUtils PUBLIC
    Qt::Core
    Qt::Network
)

target_link_directories(CommonUtils PUBLIC ${INSTALL_DIR})

if(WIN32)
    target_compile_definitions(CommonUtils PUBLIC
        WINDOWS_x86
    )

    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")


    set(COMMONUTILS_LIB ${CMAKE_CURRENT_BINARY_DIR}/CommonUtils.lib)
    set(COMMONUTILS_DLL ${CMAKE_CURRENT_BINARY_DIR}/CommonUtils.dll)
    set(COMMONUTILS_PDB ${CMAKE_CURRENT_BINARY_DIR}/CommonUtils.pdb)

    add_custom_command(TARGET CommonUtils
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${COMMONUTILS_LIB} ${INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy ${COMMONUTILS_DLL} ${INSTALL_DIR}
    )

endif()


if(UNIX)

    if(ISA_NAME STREQUAL "aarch64")
        target_compile_definitions(CommonUtils PUBLIC
            LINUX_aarch64
        )



    else()
        target_compile_definitions(CommonUtils PUBLIC
            LINUX_x86
        )

    endif()


    add_custom_command(TARGET CommonUtils
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/libCommonUtils.so" ${INSTALL_DIR}
    )
endif()



