// Implementation of connection event handler for device communication
#include "connecteventhandler.h"

DevConnSDK::ConnectEventHandler::ConnectEventHandler()
{

}

void DevConnSDK::ConnectEventHandler::onUserJoined(const char* uid, const uint64_t extraInfoPtr)
{
    Q_UNUSED(uid);
    Q_UNUSED(extraInfoPtr);
}

void DevConnSDK::ConnectEventHandler::OnUserLeave(const char* uid, const uint64_t extraInfoPtr)
{
    Q_UNUSED(uid);
    Q_UNUSED(extraInfoPtr);
}

void DevConnSDK::ConnectEventHandler::OnUserMessageReceived(const char *uid, const char *message, const int size)
{
    Q_UNUSED(uid);
    Q_UNUSED(message);
    Q_UNUSED(size);
}

void DevConnSDK::ConnectEventHandler::OnConnectionStateChanged(const ConnectionState state)
{
    Q_UNUSED(state);
}

void DevConnSDK::ConnectEventHandler::OnUserMessageSendResult(int64_t msgid, int error)
{
    Q_UNUSED(msgid);
    Q_UNUSED(error);
}

void DevConnSDK::ConnectEventHandler::OnRoomMessageSendResult(int64_t msgid, int error)
{
    Q_UNUSED(msgid);
    Q_UNUSED(error);
}

void DevConnSDK::ConnectEventHandler::OnbusinessIDConfirmResult(bool res)
{
    Q_UNUSED(res);
}

void DevConnSDK::ConnectEventHandler::OnError(int error)
{
    Q_UNUSED(error);
}
