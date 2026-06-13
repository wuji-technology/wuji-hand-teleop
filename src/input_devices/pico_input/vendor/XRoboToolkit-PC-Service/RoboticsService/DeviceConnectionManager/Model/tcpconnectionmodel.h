// TCP connection model interface for device communication
#ifndef TCPCONNECTIONMODEL_H
#define TCPCONNECTIONMODEL_H

#include <QObject>
#include <QTcpSocket>
#include "DeviceConnectionManager_global.h"
#include "connectionmodel.h"
#include "../Manage/Manage_global.h"

namespace DevConnSDK
{
class TCPConnectionModel:public DevConnSDK::ConnectionModel
{

    Q_OBJECT
    enum SpecialStatus
    {
        NORMAL = 0x00,
        SN_FOUND,
        HEART_BRAT
    };

    enum PaddingState
    {
        NoPadding = 0,
        PaddingHead,
        PaddingMsg,
        PaddingTail
    };

public:
    TCPConnectionModel()
    {
        this->m_paddingState = NoPadding;
        this->m_paddingLength = 0;
        this->m_uid = "";
        this->m_uuid = "";
        this->m_tcpPtr = nullptr;
        this->m_connectHandler = NULL;
        this->m_connectHandlerInQt = NULL;
        this->m_managerHandler = NULL;
        this->m_currentHeartBeatTime = 0;
    };

    TCPConnectionModel(const TCPConnectionModel& model)
    {
        this->m_paddingState = model.m_paddingState;
        this->m_paddingLength = model.m_paddingLength;
        this->m_dataBuffer = model.m_dataBuffer;
        this->m_uid = model.m_uid;
        this->m_uuid = model.m_uuid;
        this->m_tcpPtr = nullptr;
        this->m_connectHandler = model.m_connectHandler;
        this->m_connectHandlerInQt = model.m_connectHandlerInQt;
        this->m_managerHandler = model.m_managerHandler;
        this->m_currentHeartBeatTime = model.m_currentHeartBeatTime;
    }

    TCPConnectionModel& operator=(const TCPConnectionModel& model)
    {
        this->m_paddingState = model.m_paddingState;
        this->m_paddingLength = model.m_paddingLength;
        this->m_dataBuffer = model.m_dataBuffer;
        this->m_uid = model.m_uid;
        this->m_uuid = model.m_uuid;
        this->m_tcpPtr = model.m_tcpPtr;
        this->m_connectHandler = model.m_connectHandler;
        this->m_connectHandlerInQt = model.m_connectHandlerInQt;
        this->m_managerHandler = model.m_managerHandler;
        this->m_currentHeartBeatTime = model.m_currentHeartBeatTime;
        return *this;
    }

    int sendData(const char *data, const int size) override;
    int reciveData(const char *data, const int size) override;

    void setConnection(QTcpSocket* tcpSocket);
    void setDisconnection();

signals:
    void clientCmdSignal(const QString sn, const TcpMessage& msg);
    void offlineSignal(const QString& sn);
    void snSettedSignal(const QString& sn, const QString& tempIndex);    

public slots:
    void onReadClientMessage();
    int  dealClientPack(const TcpMessage& msg);
    void onClientMsg(const TcpMessage& msg);


public:
    QTcpSocket* m_tcpPtr;
    quint64     m_currentHeartBeatTime;
    QByteArray   m_dataBuffer;
    PaddingState m_paddingState;
    uint32_t     m_paddingLength;
};
}
#endif // TCPCONNECTIONMODEL_H
