// Implementation of TCP connection management for device communication
#include <QUuid>
#include "tcpserverworker.h"
#include "tcpconnectionmanage.h"
#include "utils/utils.h"



DevConnSDK::TcpConnectionManage::TcpConnectionManage()
{
    m_tcpServerWorker = nullptr;
}

int DevConnSDK::TcpConnectionManage::init(const uint64_t paramPtr)
{
    Q_UNUSED(paramPtr);
    qDebug() << "init tcp connection Manage." << Qt::endl;

    if(!m_tcpServerWorker)
    {
        m_tcpServerWorker = new TcpServerWorker();
    }
    m_tcpServerWorker->m_managerHandler = this;

    connect(this, &DevConnSDK::TcpConnectionManage::sendUserMessageSignal,
            m_tcpServerWorker, &DevConnSDK::TcpServerWorker::onSendUserMessage,
            Qt::ConnectionType(Qt::QueuedConnection | Qt::UniqueConnection));
    connect(this, &DevConnSDK::TcpConnectionManage::sendRoomMessageSignal,
            m_tcpServerWorker, &DevConnSDK::TcpServerWorker::onSendRoomMessage,
            Qt::ConnectionType(Qt::QueuedConnection | Qt::UniqueConnection));

    int ret = m_tcpServerWorker->init();

    m_tcpServerWorker->moveToThread(m_tcpServerWorker);

    if(ret == SDKError::NONE_ERROR)
    {
        qDebug() << "tcp server worker init success." << Qt::endl;
    }
    else
    {
        qDebug() << "tcp server worker init failed. err code: " << ret << Qt::endl;
        return ret;
    }

    return SDKError::NONE_ERROR;
}

void DevConnSDK::TcpConnectionManage::release()
{
    if(m_tcpServerWorker)
    {
        if(m_tcpServerWorker->isRunning())
        {
            m_tcpServerWorker->quit();
            m_tcpServerWorker->wait();
        }
    }

    m_connectEventHandler = NULL;
    m_connectEventHandlerInQt = NULL;
    qDebug() << "tcp connection manage release." << Qt::endl;
}


int DevConnSDK::TcpConnectionManage::setConnectEventHandler(DevConnSDK::ConnectEventHandler* handlerPtr)
{
    if(m_tcpServerWorker)
    {
        if(m_tcpServerWorker->isRunning())
        {
            qDebug() << "you should set event handler before init." << Qt::endl;
            return SDKError::TCPENGINE_SET_EVENT_HANDLER_BEFORE_INIT;
        }
    }

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

int DevConnSDK::TcpConnectionManage::setConnectEventHandlerInQt(DevConnSDK::ConnectEventHandlerInQt* handlerPtr)
{
    if(m_tcpServerWorker)
    {
        if(m_tcpServerWorker->isRunning())
        {
            qDebug() << "you should set event handler before init." << Qt::endl;
            return SDKError::TCPENGINE_SET_EVENT_HANDLER_BEFORE_INIT;
        }
    }

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




void DevConnSDK::TcpConnectionManage::sendUSerMessage(const char* uid, const char* data, const int size)
{
    QString uidStr = "";
    QByteArray dataArray;
    uidStr.append(uid);
    dataArray.append(data, size);
    emit sendUserMessageSignal(uidStr, dataArray);
}

void DevConnSDK::TcpConnectionManage::sendRoomMessage(const char* uid, const char* data, const int size)
{
    QString uidStr = "";
    QByteArray dataArray;
    uidStr.append(uid);
    dataArray.append(data, size);
    emit sendRoomMessageSignal(uidStr, dataArray);
}



