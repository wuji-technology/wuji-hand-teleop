// Implementation of base connection management functionality
#include "connectionmanage.h"
#include <QMutexLocker>

DevConnSDK::ConnectionManage::ConnectionManage()
{
    m_connectEventHandler = NULL;
    m_connectEventHandlerInQt = NULL;
}

int DevConnSDK::ConnectionManage::setConnectEventHandler(DevConnSDK::ConnectEventHandler* handlerPtr)
{
    if(m_connectEventHandlerInQt != NULL)
    {
        return SDKError::QSTYLE_CALLBACK_SETED_BEFORE;
    }
    else
    {
        m_connectEventHandler = handlerPtr;
        return SDKError::NONE_ERROR;
    }
}

int DevConnSDK::ConnectionManage::setConnectEventHandlerInQt(DevConnSDK::ConnectEventHandlerInQt* handlerPtr)
{
    if(m_connectEventHandler != NULL)
    {
        return SDKError::CALLBACK_SETED_BEFORE;
    }
    else
    {
        m_connectEventHandlerInQt = handlerPtr;
        return SDKError::NONE_ERROR;
    }
}

