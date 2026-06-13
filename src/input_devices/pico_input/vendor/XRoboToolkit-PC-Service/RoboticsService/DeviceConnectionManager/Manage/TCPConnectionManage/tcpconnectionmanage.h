// TCP connection management interface for device communication
#ifndef TCPCONNECTIONMANAGE_H
#define TCPCONNECTIONMANAGE_H

#include <QObject>
#include <QTimer>
#include <QUdpSocket>
#include <QTcpServer>
#include <QThread>
#include "./../connectionmanage.h"
#include "Model/tcpconnectionmodel.h"
#include "DeviceConnectionManager_global.h"


namespace DevConnSDK
{
class ConnectEventHandler;
class TcpServerWorker;

class TcpConnectionManage:public DevConnSDK::ConnectionManage
{
    Q_OBJECT
public:
    TcpConnectionManage();

    int init(const uint64_t paramPtr) override;
    void sendUSerMessage(const char* uid, const char* data, const int size) override;
    void sendRoomMessage(const char* uid, const char* data, const int size) override;
    void release() override;

    int setConnectEventHandler(DevConnSDK::ConnectEventHandler *handlerPtr) override;
    int setConnectEventHandlerInQt(DevConnSDK::ConnectEventHandlerInQt *handlerPtr) override;

signals:
    void sendUserMessageSignal(QString uid, QByteArray data);
    void sendRoomMessageSignal(QString uid, QByteArray data);


private:
    TcpServerWorker* m_tcpServerWorker;

};

}
#endif // TCPCONNECTIONMANAGE_H
