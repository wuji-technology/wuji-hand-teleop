//Device connection SDK header file providing interfaces for device connection management

#ifndef DEVICECONNECTIONSDK_H
#define DEVICECONNECTIONSDK_H

#include <QThread>
#include "DeviceConnectionManager_global.h"

namespace DevConnSDK
{
class ConnectionManage;
class ConnectEventHandlerInQt;
class ConnectEventHandler;

class DEVICECONNECTIONSDK_EXPORT DeviceConnectionManager
{

public:
    enum ConnectType
    {
        TcpUdp = 0,
        ByteRtc,
    };

    DeviceConnectionManager();
    ~DeviceConnectionManager();

    int initSDK(ConnectType type, const char* uid, DevConnSDK::ConnectEventHandler* handlerPtr, const uint64_t extraInfoPtr);
    int initSDK(ConnectType type, const char* uid, DevConnSDK::ConnectEventHandlerInQt* handlerPtr, const uint64_t extraInfoPtr);

    int deInitSDK();

    int registerConnectEventHandler(DevConnSDK::ConnectEventHandler* handlerPtr);
    int registerConnectEventHandlerInQt(DevConnSDK::ConnectEventHandlerInQt* handlerPtr);

    void sendUserMessage(const char* uid, const char* data, const int size);
    void sendRoomMessage(const char* uid, const char* data, const int size);

private:
    DevConnSDK::ConnectEventHandler* m_connectHandler;

    DevConnSDK::ConnectEventHandlerInQt* m_connectHandlerInQt;

    DevConnSDK::ConnectionManage* m_connectionManagePtr;

    QString m_localUid;
    ConnectType m_type;
};
}
#endif // DEVICECONNECTIONSDK_H
