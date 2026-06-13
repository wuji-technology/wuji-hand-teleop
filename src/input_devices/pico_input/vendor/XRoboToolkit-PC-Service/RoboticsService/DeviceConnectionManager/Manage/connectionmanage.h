// Base connection management interface for device communication
#ifndef CONNECTIONMANAGE_H
#define CONNECTIONMANAGE_H

#include <QObject>
#include <QThread>
#include <QHash>
#include <QReadWriteLock>
#include "EventHandler/connecteventhandler.h"
#include "EventHandler/connecteventhandlerinqt.h"


namespace DevConnSDK
{
class ConnectionManage:public QObject
{
    Q_OBJECT
public:
    ConnectionManage();

    virtual int  init(const uint64_t paramPtr) = 0;
    virtual void sendUSerMessage(const char* uid, const char* data, const int size) = 0;
    virtual void sendRoomMessage(const char* uid, const char* data, const int size) = 0;
    virtual void release() = 0;

    virtual int setConnectEventHandler(DevConnSDK::ConnectEventHandler* handlerPtr);
    virtual int setConnectEventHandlerInQt(DevConnSDK::ConnectEventHandlerInQt* handlerPtr);

public:
    QString                        m_localUid;
    ConnectEventHandler*           m_connectEventHandler;
    ConnectEventHandlerInQt*       m_connectEventHandlerInQt;
};
}




#endif // CONNECTIONMANAGE_H
