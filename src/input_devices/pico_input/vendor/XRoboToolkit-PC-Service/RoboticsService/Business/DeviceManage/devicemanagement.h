// Device management class for handling device connections and communications
#ifndef DEVICEMANAGEMENT_H
#define DEVICEMANAGEMENT_H

#include <QVector>
#include <QTimer>
#include <QByteArray>


#include "DeviceConnectionManager.h"
#include "EventHandler/connecteventhandlerinqt.h"
#include "Model/Device/devicemodel.h"

#define UNUSED_INDEX 0x00
#define USED_INDEX 0x01
#define MAX_DEVICE_NUM 1000

#define PLAYINFO_SOURCE_TYPE_PC     1
#define PLAYINFO_SOURCE_TYPE_PAD    2
#define PLAYINFO_SOURCE_TYPE_SDK    3


class BUSINESSSHARED_EXPORT DeviceManagement:public QObject
{
    Q_OBJECT

public:
    DeviceManagement();
    ~DeviceManagement();

    void parseSetting();



signals:

    void newDeviceConnectSignal(DeviceModel&);
    void rtcUidRepeatSignal(const QString& uid);
    void pcJoinRoomSignal(const QString& uid);
    void newUserJoinedSiganl(QString uid);
    void openVRShareTexFailed();
    void rtcDisconnectedSignal();
    void deviceLeaveRoomSignal(QString sn);
    void connectStatusSignal(bool flag);
    void roomStatusSignal(bool flag,QString serverIP);

    void updateConnectedDeviceListSignal(QVector<DeviceModel> array);
    void updateAddedDeviceList(QVector<DeviceModel> array);
    void updateAddedDeviceSignal(QString jsonStr);
    void updateDeviceGroupSignal(QString jsonStr);
    void setServerUidReponseSignal(const QString& sn, bool flag, const QString& lockIP);
    void newDeviceAdded(const QString& sn);
    void removeAddDevcieSignal(const QString& sn);
    void filterGroupChanged(const QString& str);
    void deliverMsgToBroadCastSignal(const QString& uid, const TcpMessage& tcpMessage);
    void deliverMsgToVideoLiveSignal(const QString& uid, const TcpMessage& tcpMessage);
    void businessIDConfirmResultSignal(bool result);
    void setUidSignal(QString uid);
    void replyExistDevice(QString devid);
    void recvDeviceMessageSignal(QString devid,int type,QByteArray msgbody);

    void replyFirstMonitorScreenFrame(QString devid, int width, int height);
    void replyMonitorScreenFrame(QString devid, QByteArray rtcFrame, int width, int height, int bytesPerLine, int pixFormat);
    void replyFirstMonitorStreamFrame(QString devid, int width, int height);
    void replyMonitorStreamFrame(QString devid, QByteArray rtcFrame, int width, int height, int bytesPerLine, int pixFormat);


public:
    void ReplyOnlineDevice();


public slots:
    quint64 getThisPoint()
    {
        return (quint64)this;
    }

    void init();
    void initTcp();

    void sendMsg(QString sn, const QByteArray& data);
    void sendMsg(QString sn, const TcpMessage& msg);
    void sendRoomMsg(const TcpMessage& msg);
    void onNewDeviceMessageSDK(QString uid, int size, QByteArray dataArray);

    //rtc msg
    void onNewRtcConnection(QString sn);
    void onRtcDeviceOffLine(QString sn);

    //send msg
    void sendDeviceControlJson(const QString& sn, const QString& parameterJson);
    void sendBytesToDevice(QString sn, QByteArray data);
    void sendBytesToRoom(QByteArray data);


private:
    void setLocalUid();
    int  getModelID();
    void ReplySDKClient(const QString& uid, const TcpMessage& tcpMessage);


public:
    DevConnSDK::ConnectEventHandlerInQt m_connectEvent;
    DevConnSDK::DeviceConnectionManager  m_deviceConnectSDK;
    QMap<QString, DeviceModel>       m_connectDevice;
private:
    QString                          m_localUid;
    unsigned char                    m_indexSlot[MAX_DEVICE_NUM];
    bool                             m_requiresUpdateDevice;
    bool                             m_requiresUpdateGroup;
};

#endif // DEVICEMANAGEMENT_H
