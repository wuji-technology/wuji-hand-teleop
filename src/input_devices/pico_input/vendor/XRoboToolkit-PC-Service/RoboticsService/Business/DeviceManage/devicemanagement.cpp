#include <QCoreApplication>
#include <QSettings>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QList>
#include <QUuid>
#include <QRandomGenerator>
#include <QStandardPaths>
#include "devicemanagement.h"
#include "devicemanagement_msg.h"
#include "commonutils.h"
#include "../../CommonUtils/OSFileID.h"

DeviceManagement::DeviceManagement()
{
    m_localUid = "";
}

DeviceManagement::~DeviceManagement()
{
}




void DeviceManagement::parseSetting()
{

}

void DeviceManagement::ReplyOnlineDevice()
{
    for (QMap<QString, DeviceModel>::const_iterator iter = m_connectDevice.constBegin();
         iter != m_connectDevice.constEnd();
         ++iter)
    {
        if(iter.value().m_isOnline)
        {
            qDebug() <<"reply existing device"<<iter.key()<< Qt::endl;
            emit replyExistDevice(iter.key());
        }
    }
}



void DeviceManagement::initTcp()
{
    QByteArray uidArray = m_localUid.toUtf8();
    m_deviceConnectSDK.initSDK(DevConnSDK::DeviceConnectionManager::TcpUdp, uidArray.data(), &m_connectEvent, NULL);
}





void DeviceManagement::onNewRtcConnection(QString sn)
{
    if(sn == "")
    {
        qDebug() << "receive empty sn!" << Qt::endl;
        return;
    }
    //emit m_rtc.replyDeviceFind(sn);
    if(m_connectDevice.contains(sn))
    {
        m_connectDevice[sn].m_isOnline = true;
        m_connectDevice[sn].m_state = DeviceModel::DeviceState::FreeState;
        LOG_INFO("rtc device already added, uid: " + sn);
    }
    else
    {
        LOG_INFO("new RTC device connected: " + sn);
        DeviceModel model;
        model.m_sn = sn;
        model.m_uid = sn;
        model.m_isOnline = true;
        model.m_state = DeviceModel::DeviceState::FreeState;
        model.m_id = "";
        m_connectDevice.insert(sn, model);
    }

}

void DeviceManagement::onRtcDeviceOffLine(QString sn)
{
    if(m_connectDevice.contains(sn))
    {
        LOG_INFO("rtc device offline, uid: " + sn);
        m_connectDevice[sn].m_isOnline = false;
        m_connectDevice[sn].m_state = DeviceModel::DeviceState::OffLineState;
        m_connectDevice[sn].m_isShared = false;
        m_connectDevice[sn].m_isPlay = false;
        m_connectDevice[sn].m_isSelected = false;
        m_connectDevice[sn].m_isLock = false;
        m_connectDevice[sn].m_isMyLock = false;
        m_connectDevice[sn].m_isCalling = false;
    }
    else
    {
        qDebug() << "unknow rtc device offline, uid: " << sn << Qt::endl;
        DeviceModel model;
        model.m_sn = sn;
        model.m_uid = sn;
        model.m_isOnline = false;
        model.m_state = DeviceModel::DeviceState::OffLineState;
        model.m_id = QString::number(getModelID());
        model.m_isShared = false;
        model.m_isPlay = false;
        model.m_isSelected = false;
        model.m_isLock = false;
        model.m_isMyLock = false;
        m_connectDevice.insert(sn, model);
    }
    emit deviceLeaveRoomSignal(sn);
}

void DeviceManagement::init()
{
    parseSetting();
    setLocalUid();
    connect(&m_connectEvent, SIGNAL(userJoinedSignal(QString)), this, SLOT(onNewRtcConnection(QString)));
    connect(&m_connectEvent, SIGNAL(userLeaveSignal(QString)), this, SLOT(onRtcDeviceOffLine(QString)));
    connect(&m_connectEvent, &DevConnSDK::ConnectEventHandlerInQt::connectStatusSignal, this, &DeviceManagement::connectStatusSignal);
    connect(&m_connectEvent, SIGNAL(userBinaryMsgSignal(QString,int,QByteArray)), this, SLOT(onNewDeviceMessageSDK(QString,int,QByteArray)));

    qDebug() << "init tcp protocol procedure..." << Qt::endl;
    initTcp();
 }


void DeviceManagement::sendMsg(QString sn, const QByteArray& data)
{
    QByteArray snArray = sn.toUtf8();
    m_deviceConnectSDK.sendUserMessage(snArray.data(), data.data(), data.size());
}

void DeviceManagement::sendMsg(QString sn, const TcpMessage& msg)
{
    qDebug() << "send sn: " << sn << " cmd: " << msg.m_head.cmd << " msg: " << QString::fromUtf8(msg.m_msg) << Qt::endl;

    QByteArray snArray = sn.toUtf8();
    QByteArray dataArray = msg.toByteArray();

    m_deviceConnectSDK.sendUserMessage(snArray.data(), dataArray.data(), dataArray.size());
}

void DeviceManagement::sendRoomMsg(const TcpMessage& msg)
{
    qDebug() << "send to room, cmd: " << msg.m_head.cmd << " msg: " << msg.m_msg << Qt::endl;
    QByteArray dataArray = msg.toByteArray();
    m_deviceConnectSDK.sendRoomMessage("", dataArray.data(), dataArray.size());
}


void DeviceManagement::sendDeviceControlJson(const QString& sn, const QString& parameterJson)
{
    sendMsg(sn, TcpMessage(TCP_SERVER_CMD_DEVICE_CONTROL_JSON, parameterJson));
}

void DeviceManagement::sendBytesToDevice(QString sn, QByteArray data)
{
    sendMsg(sn, TcpMessage(TCP_SERVER_CMD_SEND_BYTES_TO_DEVICE, data));
}

void DeviceManagement::sendBytesToRoom(QByteArray data)
{
    sendRoomMsg(TcpMessage(TCP_SERVER_CMD_SEND_BYTES_TO_DEVICE, data));
}

void DeviceManagement::onNewDeviceMessageSDK(QString uid, int size, QByteArray dataArray)
{
    if(uid == "")
    {
        qDebug() << "receive empty uid message! data: " << dataArray << Qt::endl;
        return;
    }


    if (m_connectDevice.find(uid) == m_connectDevice.end())
    {
        onNewRtcConnection(uid);
    }
    else
    {
        if(m_connectDevice[uid].m_isOnline == false)
        {
            onNewRtcConnection(uid);
        }
    }
    TcpMessage::parseByteArray(size,dataArray.data(),[uid,this](TcpMessage& tcpMsg){
        ReplySDKClient(uid,tcpMsg);
    });
}

int DeviceManagement::getModelID()
{
    int idx;
    for(idx=1; idx<MAX_DEVICE_NUM; ++idx)
    {
        if(m_indexSlot[idx] == UNUSED_INDEX)
        {
            qDebug() <<"allocate index"<<idx<< Qt::endl;
            m_indexSlot[idx] = USED_INDEX;
            return idx;
        }
    }

    LOG_INFO("no idle index, choose existing index");
    idx = QRandomGenerator::global()->bounded(1, MAX_DEVICE_NUM);
    qDebug() <<"allocate used index"<<idx<< Qt::endl;
    return idx;
}

void DeviceManagement::ReplySDKClient(const QString& uid, const TcpMessage& tcpMessage)
{
    //qDebug() << "sdk recive msg uid:" << uid << " cmd: " << tcpMessage.m_head.cmd  << " msg:" << tcpMessage.m_msg  << Qt::endl;
    switch (tcpMessage.m_head.cmd)
    {

    case TCP_CLIENT_MSG_CONNECT:
    {
        QList<QByteArray> parts = tcpMessage.m_msg.split('|');
        qDebug() <<"device connect"<<QString::fromUtf8(parts[0])<<QString::fromUtf8(parts[1])<< Qt::endl;
        emit recvDeviceMessageSignal(QString::fromUtf8(parts[0]),PXREAServerMsgConnect,parts[1]);
    }
        break;
    case TCP_CLIENT_MSG_DEVICE_STATE_JSON:
    {
        //qDebug() <<"device state json"<<QString::fromUtf8(tcpMessage.m_msg)<< Qt::endl;
        emit recvDeviceMessageSignal(uid,PXREAServerMsgDeviceStateJson,tcpMessage.m_msg);
    }
        break;
    case TCP_CLIENT_MSG_SEND_BYTES_TO_DEVICE:
    {
        // 直接使用原始的二进制数据
        emit recvDeviceMessageSignal(uid,PXREAServerSendCustomMessage,tcpMessage.m_msg);
    }
        break;
    }
}


void DeviceManagement::setLocalUid()
{
    QSettings appSetting("setting.ini",QSettings::IniFormat);
    //uid body
    QString uid = appSetting.value("DeviceControl/UID").toString();
    if(uid.size() == 0)
    {
        QString ip = CommonUtils::getLocalIPv4Address();
        if(ip.size() > 0)
        {
            uid = ip.replace('.', '_');
        }
        else
        {
            QString uuid = QUuid::createUuid().toString();
            uid = uuid.mid(1, uuid.toLocal8Bit().size() - 2);
        }
    }
    else
    {
        //user define uid
    }

    //add uid prefix
    bool showInRTCRoom = appSetting.value("DeviceControl/showInRTCRoom", false).toBool();
    QString prefix = showInRTCRoom ? "SHOW_" : "PA_";
    m_localUid = prefix + uid;

    emit setUidSignal(m_localUid);
}
