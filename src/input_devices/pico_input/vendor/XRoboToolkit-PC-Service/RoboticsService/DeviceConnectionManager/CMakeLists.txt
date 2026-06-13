# Generated from DeviceConnectionManager.pro.
cmake_minimum_required(VERSION 3.16)
project(DeviceConnectionManager VERSION 1.0)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)

set(INSTALL_DIR  ${PROJECT_SOURCE_DIR}/../bin)

if(NOT EXISTS ${INSTALL_DIR})
    file(MAKE_DIRECTORY ${INSTALL_DIR})
endif()

find_package(QT NAMES Qt5 Qt6 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Network)


qt_add_library(DeviceConnectionManager SHARED
    DeviceConnectionManager_global.h
    EventHandler/connecteventhandler.h
    EventHandler/connecteventhandlerinqt.h
    Manage/Manage_global.h
    Manage/TCPConnectionManage/tcpconnectionmanage.h
    Manage/TCPConnectionManage/tcpserverworker.h
    Manage/connectionmanage.h
    Model/connectionmodel.h
    Model/tcpconnectionmodel.h
    DeviceConnectionManager.h
    utils/utils.h
    EventHandler/connecteventhandler.cpp
    EventHandler/connecteventhandlerinqt.cpp
    Manage/TCPConnectionManage/tcpconnectionmanage.cpp
    Manage/TCPConnectionManage/tcpserverworker.cpp
    Manage/connectionmanage.cpp
    Model/connectionmodel.cpp
    Model/tcpconnectionmodel.cpp
    DeviceConnectionManager.cpp
    utils/utils.cpp
)



target_compile_definitions(DeviceConnectionManager PUBLIC
    DEVICECONNECTIONSDK_LIBRARY
    QT_DEPRECATED_WARNINGS
)

target_link_libraries(DeviceConnectionManager PUBLIC
    Qt::Core
    Qt::Network
)

target_link_directories(DeviceConnectionManager PUBLIC ${INSTALL_DIR})

if(WIN32)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

    target_include_directories(DeviceConnectionManager PUBLIC
        ../CommonUtils
    )

    target_compile_definitions(DeviceConnectionManager PUBLIC
        WINDOWS_x86
    )

    target_link_libraries(DeviceConnectionManager PUBLIC
        # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../Redistributable/win/openvr/lib/win64
        # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
        # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/ByteRTC/
        CommonUtils.dll
    )

    set(DEVICECONNECTIONSDK_LIB ${CMAKE_CURRENT_BINARY_DIR}/DeviceConnectionManager.lib)
    set(DEVICECONNECTIONSDK_DLL ${CMAKE_CURRENT_BINARY_DIR}/DeviceConnectionManager.dll)
    set(DEVICECONNECTIONSDK_PDB ${CMAKE_CURRENT_BINARY_DIR}/DeviceConnectionManager.pdb)

    add_custom_command(TARGET DeviceConnectionManager
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${DEVICECONNECTIONSDK_LIB} ${INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy ${DEVICECONNECTIONSDK_DLL} ${INSTALL_DIR}
    )

endif()


if(UNIX)
    target_include_directories(DeviceConnectionManager PUBLIC
        ../CommonUtils
    )

    if(ISA_NAME STREQUAL "aarch64")
        target_compile_definitions(DeviceConnectionManager PUBLIC
            LINUX_aarch64
        )

        target_link_libraries(DeviceConnectionManager PUBLIC
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/ByteRTC/
            CommonUtils
        )
    else()
        target_compile_definitions(DeviceConnectionManager PUBLIC
            LINUX_x86
        )

        target_link_libraries(DeviceConnectionManager PUBLIC
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/
            # Remove: L${CMAKE_CURRENT_SOURCE_DIR}/../bin/ByteRTC/
            CommonUtils
        )

    endif()
    add_custom_command(TARGET DeviceConnectionManager
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/libDeviceConnectionManager.so" ${INSTALL_DIR}
    )

endif()

