// TCP server worker interface for handling device connections
#ifndef TCPSERVERWORKER_H
#define TCPSERVERWORKER_H

#include <QObject>
#include <QTimer>
#include <QUdpSocket>
#include <QTcpServer>
#include <QThread>
#include "Model/tcpconnectionmodel.h"

namespace DevConnSDK
{

class TcpConnectionManage;

class TcpServerWorker : public QThread
{
    Q_OBJECT
public:
    TcpServerWorker();
    ~TcpServerWorker();

    int  init();
    void release();


protected:
    void run() override;


public slots:
    void onCheckConnection();
    void onBraodCastTCPServer();
    void onNewTcpConnection();

    void onSendUserMessage(QString uid, QByteArray data);
    void onSendRoomMessage(QString uid, QByteArray data);


    void onNewTcpDeviceConfirmed(const QString& sn, const QString& tempIndex);


public:
    QMap<QString, TCPConnectionModel>  m_connectDevice;
    TcpConnectionManage*               m_managerHandler;
    QTcpServer*                        m_tcpServer;
    QMap<QString, TCPConnectionModel>  m_tempTcpMap;
    QUdpSocket*                        m_broadCastUdp;
    QTimer*                            m_tcpBroadcastTimer;
    QTimer*                            m_connectCheckTimer;

private:
    int m_udpBroadCastPort;
    int m_tcpBindPort;
};
}

#endif // TCPSERVERWORKER_H
