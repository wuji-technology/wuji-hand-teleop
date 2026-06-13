// Implementation of DeviceConnectionManager core functionality
#include "DeviceConnectionManager.h"
#include "EventHandler/connecteventhandler.h"
#include "EventHandler/connecteventhandlerinqt.h"
#include "Manage/TCPConnectionManage/tcpconnectionmanage.h"


DevConnSDK::DeviceConnectionManager::DeviceConnectionManager()
{
    m_localUid = "";

    m_connectHandler = NULL;

    m_connectHandlerInQt = NULL;

    m_connectionManagePtr = NULL;
}

DevConnSDK::DeviceConnectionManager::~DeviceConnectionManager()
{
    if(m_connectionManagePtr)
    {
        m_connectionManagePtr->release();
        delete m_connectionManagePtr;
        m_connectionManagePtr = NULL;
    }
}

int DevConnSDK::DeviceConnectionManager::deInitSDK()
{
    if(m_connectionManagePtr)
    {
        m_connectionManagePtr->release();
        delete m_connectionManagePtr;
        m_connectionManagePtr = NULL;
    }
    return SDKError::NONE_ERROR;
}

int DevConnSDK::DeviceConnectionManager::initSDK(DevConnSDK::DeviceConnectionManager::ConnectType type, const char* uid, DevConnSDK::ConnectEventHandler* handlerPtr, const uint64_t extraInfoPtr)
{
    m_localUid = "";
    m_localUid.append(uid);
    m_type = type;
    switch (type)
    {
        case TcpUdp:
        {
            m_connectionManagePtr = new TcpConnectionManage();
            break;
        }
        case ByteRtc:
        {
            return DevConnSDK::SDKError::PLATFORM_NOT_SUPOORT;
        }
        default:
            return DevConnSDK::SDKError::UNKNOWN_CONNECT_TYPE;

    }

    m_connectionManagePtr->setConnectEventHandler(handlerPtr);
    m_connectionManagePtr->m_localUid = m_localUid;
    return m_connectionManagePtr->init(extraInfoPtr);
}

int DevConnSDK::DeviceConnectionManager::initSDK(ConnectType type, const char* uid, DevConnSDK::ConnectEventHandlerInQt* handlerPtr, const uint64_t extraInfoPtr)
{
    m_localUid = "";
    m_localUid.append(uid);
    m_type = type;
    switch (type)
    {
        case TcpUdp:
        {
             m_connectionManagePtr = new TcpConnectionManage();
             break;
        }
        case ByteRtc:
        {
            return DevConnSDK::SDKError::PLATFORM_NOT_SUPOORT;
        }
        default:
            return DevConnSDK::SDKError::UNKNOWN_CONNECT_TYPE;

    }

    m_connectionManagePtr->setConnectEventHandlerInQt(handlerPtr);
    m_connectionManagePtr->m_localUid = m_localUid;
    return m_connectionManagePtr->init(extraInfoPtr);
}

void DevConnSDK::DeviceConnectionManager::sendUserMessage(const char* uid, const char* data, int size)
{
    if(m_connectionManagePtr == NULL)
    {
        qWarning() << "SDK_NOT_INIT." << Qt::endl;
        return;
    }
    m_connectionManagePtr->sendUSerMessage(uid, data, size);
}

void DevConnSDK::DeviceConnectionManager::sendRoomMessage(const char* uid, const char* data, int size)
{
    if(m_connectionManagePtr == NULL)
    {
        qWarning() << "SDK_NOT_INIT." << Qt::endl;
        return;
    }
    m_connectionManagePtr->sendRoomMessage(uid, data, size);
}

int DevConnSDK::DeviceConnectionManager::registerConnectEventHandler(DevConnSDK::ConnectEventHandler* handlerPtr)
{
    if(m_connectHandlerInQt != NULL)
    {
        return SDKError::QSTYLE_CALLBACK_SETED_BEFORE;
    }
    else
    {
        m_connectHandler = handlerPtr;
        if(m_connectionManagePtr != NULL)
        {
            return m_connectionManagePtr->setConnectEventHandler(handlerPtr);
        }

        return SDKError::NONE_ERROR;
    }
}

int DevConnSDK::DeviceConnectionManager::registerConnectEventHandlerInQt(DevConnSDK::ConnectEventHandlerInQt* handlerPtr)
{
    if(m_connectHandler != NULL)
    {
        return SDKError::CALLBACK_SETED_BEFORE;
    }
    else
    {
        m_connectHandlerInQt = handlerPtr;
        if(m_connectionManagePtr != NULL)
        {
            m_connectionManagePtr->setConnectEventHandlerInQt(handlerPtr);
        }

        return SDKError::NONE_ERROR;
    }
}
