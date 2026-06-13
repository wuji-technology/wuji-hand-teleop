// Event handler interface for device connection events
#ifndef CONNECTEVENTHANDLER_H
#define CONNECTEVENTHANDLER_H

#include <QObject>
#include "../DeviceConnectionManager_global.h"

namespace DevConnSDK
{
class DEVICECONNECTIONSDK_EXPORT ConnectEventHandler
{

public:
    ConnectEventHandler();
    //extraInfoPtr default value is null
    virtual void onUserJoined(const char* uid, const uint64_t extraInfoPtr);
    virtual void OnUserLeave(const char* uid, const uint64_t extraInfoPtr);
    virtual void OnUserMessageReceived(const char* uid, const char* message,  const int size);
    virtual void OnConnectionStateChanged(const ConnectionState state);
    virtual void OnUserMessageSendResult(int64_t msgid, int error);
    virtual void OnRoomMessageSendResult(int64_t msgid, int error);
    virtual void OnbusinessIDConfirmResult(bool res);
    virtual void OnError(int);
};
}

#endif // CONNECTEVENTHANDLER_H
