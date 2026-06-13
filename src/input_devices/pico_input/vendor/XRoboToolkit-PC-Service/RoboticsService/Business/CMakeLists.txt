# Generated from Business.pro.

cmake_minimum_required(VERSION 3.16)
project(Business VERSION 1.0 LANGUAGES CXX)
 
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)

set(INSTALL_DIR  ${PROJECT_SOURCE_DIR}/../bin)

if(NOT EXISTS ${INSTALL_DIR})
    file(MAKE_DIRECTORY ${INSTALL_DIR})
endif()


set(CMAKE_CXX_STANDARD 17)

find_package(Qt6 REQUIRED COMPONENTS Core5Compat)
find_package(QT NAMES Qt5 Qt6 REQUIRED COMPONENTS Core)




qt_add_library(Business SHARED
    Business_global.h
    DeviceManage/devicemanagement.cpp DeviceManage/devicemanagement.h
    DeviceManage/devicemanagement_msg.h
    Model/Device/devicemodel.cpp Model/Device/devicemodel.h
    business.cpp business.h
)


target_include_directories(Business PUBLIC
    ../CommonUtils
    ../DeviceConnectionManager
)


target_compile_definitions(Business PUBLIC
    BUSINESS_LIBRARY
    QT_MESSAGELOGCONTEXT
    _HAS_STD_BYTE=1
)

target_link_libraries(Business PRIVATE Qt6::Core5Compat)

target_link_libraries(Business PUBLIC
    Qt::Core
)

target_link_directories(Business PUBLIC ${INSTALL_DIR})

if(WIN32)
    target_compile_definitions(Business PUBLIC
        WINDOWS_x86
    )
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")



    target_link_libraries(Business PUBLIC
        # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
        # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/ByteRTC/
        CommonUtils.dll
        DeviceConnectionManager.dll
    )


    set(BUSINESS_LIB ${CMAKE_CURRENT_BINARY_DIR}/Business.lib)
    set(BUSINESS_DLL ${CMAKE_CURRENT_BINARY_DIR}/Business.dll)
    set(BUSINESS_PDB ${CMAKE_CURRENT_BINARY_DIR}/Business.pdb)

    add_custom_command(TARGET Business
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${BUSINESS_LIB} ${INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy ${BUSINESS_DLL} ${INSTALL_DIR}
    )


endif()


if(UNIX)
    if(ISA_NAME STREQUAL "aarch64")
        target_compile_definitions(Business PUBLIC
            LINUX_aarch64
        )

        target_link_libraries(Business PUBLIC
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
            CommonUtils
            DeviceConnectionManager
        )
    else()
        target_compile_definitions(Business PUBLIC
            LINUX_x86
        )

        target_link_libraries(Business PUBLIC
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
            CommonUtils
            DeviceConnectionManager
        )
    endif()
    add_custom_command(TARGET Business
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/libBusiness.so" ${INSTALL_DIR}
    )

endif()

