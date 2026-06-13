// Implementation of TCP server worker for handling device connections
#include <QUuid>
#include <QSettings>
#include <QCoreApplication>
#include <QStandardPaths>
#include "tcpconnectionmanage.h"
#include "tcpserverworker.h"
#include "utils/utils.h"

DevConnSDK::TcpServerWorker::TcpServerWorker()
{
    m_tcpServer = nullptr;
    m_broadCastUdp = nullptr;
    m_tcpBroadcastTimer = nullptr;
    m_connectCheckTimer = nullptr;
    m_managerHandler = nullptr;
    m_udpBroadCastPort = BROADCAST_UDP_PORT;
    m_tcpBindPort = DEFAULT_TCP_PORT;
}

DevConnSDK::TcpServerWorker::~TcpServerWorker()
{
}

void DevConnSDK::TcpServerWorker::release()
{
    if(m_tcpServer)
    {
        m_tcpServer->close();
        delete m_tcpServer;
        m_tcpServer = nullptr;
    }

    if(m_broadCastUdp)
    {
        delete m_broadCastUdp;
        m_broadCastUdp = nullptr;
    }

    if(m_tcpBroadcastTimer)
    {
        m_tcpBroadcastTimer->stop();
        delete m_tcpBroadcastTimer;
        m_tcpBroadcastTimer = nullptr;
    }

    if(m_connectCheckTimer)
    {
        m_connectCheckTimer->stop();
        delete m_connectCheckTimer;
        m_connectCheckTimer = nullptr;
    }
}


void DevConnSDK::TcpServerWorker::run()
{
    m_connectDevice.clear();
    m_tempTcpMap.clear();

    if(!m_tcpServer)
    {
        m_tcpServer = new QTcpServer();
    }

    if(!m_tcpServer->listen(QHostAddress::Any, m_tcpBindPort))
    {
        qDebug() << "tcp server listen failed. port: " << m_tcpBindPort << Qt::endl;
        return;
    }

    connect(m_tcpServer, SIGNAL(newConnection()), this, SLOT(onNewTcpConnection()), Qt::UniqueConnection);

    if(!m_broadCastUdp)
    {
        m_broadCastUdp = new QUdpSocket();
    }

    if(!m_tcpBroadcastTimer)
    {
        m_tcpBroadcastTimer = new QTimer();
    }
    connect(m_tcpBroadcastTimer, SIGNAL(timeout()), this, SLOT(onBraodCastTCPServer()), Qt::UniqueConnection);
    m_tcpBroadcastTimer->start(BROADCAST_MEADIA_SERVER_INTERVAL);

    if(!m_connectCheckTimer)
    {
        m_connectCheckTimer = new QTimer();
    }
    connect(m_connectCheckTimer, SIGNAL(timeout()), this, SLOT(onCheckConnection()), Qt::UniqueConnection);
    m_connectCheckTimer->start(TCP_CONNECTION_CHECK_INTERVAL);

    this->exec();
    release();
}

int DevConnSDK::TcpServerWorker::init()
{

    QString applicationPath = QCoreApplication::applicationDirPath();
    QString settingPath = applicationPath + "/setting.ini";
    QSettings settings(settingPath, QSettings::IniFormat);

    m_udpBroadCastPort = settings.value("TCP/BroadCastSendPort", BROADCAST_UDP_PORT).toInt();
    m_tcpBindPort = settings.value("TCP/TcpBindPort", DEFAULT_TCP_PORT).toInt();

    if(!m_managerHandler)
    {
        qDebug() << "manager handler not set." << Qt::endl;
        return SDKError::TCPENGINE_SERVER_ERROR;
    }

    this->start();

    return SDKError::NONE_ERROR;
}

void DevConnSDK::TcpServerWorker::onBraodCastTCPServer()
{
    if(!m_broadCastUdp)
    {
        qDebug() << "udp socket ptr is null." << Qt::endl;
        return;
    }

    QStringList ipList = Utils::getLocalIPv4AddressList();

    foreach(const QString& ip, ipList)
    {
        QString data = ip;
        data = data.replace(ip.lastIndexOf('.') + 1, 3, "255");
        qDebug() << data <<Qt::endl;
        TcpMessage msg;
        msg.m_head.cmd = UDP_PACKET_CMD_TCPIP;
        msg.m_head.length = ip.toLocal8Bit().length();
        msg.m_head.head = TCP_SERVER_HEAD_CODE;
        msg.m_msg = ip.toLocal8Bit();
        msg.m_tail.timeStamp = QDateTime::currentSecsSinceEpoch();
        msg.m_tail.tail = TCP_CLIENT_MSG_TAIL_CODE;

       if(m_broadCastUdp->writeDatagram(msg.toByteArray(), QHostAddress(data), m_udpBroadCastPort) < 0)
       {
           qDebug() << "onBraodCastMediaServer error!" << Qt::endl;
       }
    }
}

void DevConnSDK::TcpServerWorker::onNewTcpConnection()
{
    if(!m_managerHandler)
    {
        qDebug() << "tcp manager ptr is null." << Qt::endl;
        return;
    }

    DevConnSDK::TCPConnectionModel model;
    QTcpSocket* tcpSocket = m_tcpServer->nextPendingConnection();

    QString uuid = QUuid::createUuid().toString();

    m_tempTcpMap.insert(uuid, model);
    m_tempTcpMap[uuid].m_uuid = uuid;
    m_tempTcpMap[uuid].setConnection(tcpSocket);
    connect(&m_tempTcpMap[uuid], SIGNAL(snSettedSignal(const QString&, const QString&)),
            this, SLOT(onNewTcpDeviceConfirmed(const QString&, const QString&)), Qt::UniqueConnection);
}

void DevConnSDK::TcpServerWorker::onNewTcpDeviceConfirmed(const QString& sn, const QString& tempIndex)
{
    qDebug() << "new tcp connect device: " << sn << Qt::endl;
    if (!m_connectDevice.contains(sn))
    {
        TCPConnectionModel model;
        model.m_uid = sn;
        model.m_connectHandler = m_managerHandler->m_connectEventHandler;
        model.m_connectHandlerInQt = m_managerHandler->m_connectEventHandlerInQt;
        model.m_managerHandler = m_managerHandler;

        m_connectDevice.insert(sn, model);

        if(!m_tempTcpMap.contains(tempIndex))
        {
            qDebug() << "invalid temp index: " << tempIndex << " tcpSocket would not set." << Qt::endl;
        }
        else
        {
            qDebug() << "new tcp connect device ddd to map: " << sn << Qt::endl;
            m_connectDevice[model.m_uid].setConnection(m_tempTcpMap[tempIndex].m_tcpPtr);
            m_tempTcpMap[tempIndex].setDisconnection();
        }

        if(m_managerHandler->m_connectEventHandler != NULL)
        {
            m_managerHandler->m_connectEventHandler->onUserJoined(sn.toLocal8Bit().data(), NULL);
        }
        else if(m_managerHandler->m_connectEventHandlerInQt != NULL)
        {
            emit m_managerHandler->m_connectEventHandlerInQt->userJoinedSignal(sn);
        }
        else
        {}
    }
    else
    {
        qDebug() << "new tcp connect device added before: " << sn << Qt::endl;
        if(!m_tempTcpMap.contains(tempIndex))
        {
            qDebug() << "invalid temp index: " << tempIndex << " tcpSocket would not set." << Qt::endl;
        }
        else
        {
            m_connectDevice[sn].setDisconnection();
            m_connectDevice[sn].setConnection(m_tempTcpMap[tempIndex].m_tcpPtr);
            m_tempTcpMap[tempIndex].setDisconnection();
        }
    }
}

void DevConnSDK::TcpServerWorker::onSendUserMessage(QString uid, QByteArray data)
{
    if(!m_connectDevice.contains(uid))
    {
        qDebug() << "uid not exist: " << uid << Qt::endl;
        return;
    }
    m_connectDevice[uid].sendData(data.data(), data.size());
}

void DevConnSDK::TcpServerWorker::onSendRoomMessage(QString uid, QByteArray data)
{
    Q_UNUSED(uid);
    QMap<QString, TCPConnectionModel>::const_iterator iter = m_connectDevice.constBegin();

    while (iter != m_connectDevice.constEnd())
    {
        m_connectDevice[iter.key()].sendData(data.data(), data.size());
        iter++;
    }
}

void DevConnSDK::TcpServerWorker::onCheckConnection()
{
    if(!m_managerHandler)
    {
        return;
    }

    QStringList offlineList;
    QMap<QString, TCPConnectionModel>::const_iterator iter = m_connectDevice.constBegin();
    quint64 secs = QDateTime::currentSecsSinceEpoch();
    while (iter != m_connectDevice.constEnd())
    {
        if(iter->m_currentHeartBeatTime == 0)
        {
            iter++;
            continue;
        }
        else
        {
            if(secs - iter->m_currentHeartBeatTime > 20)
            {
                offlineList << iter->m_uid;
                if(m_managerHandler->m_connectEventHandler != NULL)
                {
                    m_managerHandler->m_connectEventHandler->OnUserLeave(iter->m_uid.toLocal8Bit().data(), NULL);
                }
                else if(m_managerHandler->m_connectEventHandlerInQt != NULL)
                {
                    emit m_managerHandler->m_connectEventHandlerInQt->userLeaveSignal(iter->m_uid);
                }
                else
                {}
            }
            else
            {}
        }

        iter++;
    }

    foreach(const QString& uid, offlineList)
    {
        if(m_connectDevice[uid].m_tcpPtr)
        {
            qDebug() << uid << "remove tcp." << Qt::endl;
            m_connectDevice[uid].setDisconnection();
            m_connectDevice[uid].m_tcpPtr = nullptr;
        }
        m_connectDevice.remove(uid);
        qDebug() << uid << " disconnect." << Qt::endl;
    }
}




