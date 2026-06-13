// Base connection model interface for device communication
#ifndef CONNECTIONMODEL_H
#define CONNECTIONMODEL_H

#include <QObject>
#include "EventHandler/connecteventhandler.h"
#include "EventHandler/connecteventhandlerinqt.h"
#include "Manage/connectionmanage.h"

namespace DevConnSDK
{
class ConnectionModel: public QObject
{
    Q_OBJECT
public:    
    ConnectionModel()
    {
        this->m_uid = "";
        this->m_uuid = "";
        this->m_connectHandler = NULL;
        this->m_connectHandlerInQt = NULL;
        this->m_managerHandler = NULL;
    };

    ConnectionModel(const ConnectionModel& model)
    {
        this->m_uid = model.m_uid;
        this->m_uuid = model.m_uuid;
        this->m_connectHandler = model.m_connectHandler;
        this->m_connectHandlerInQt = model.m_connectHandlerInQt;
        this->m_managerHandler = model.m_managerHandler;
    }

    ConnectionModel& operator=(const ConnectionModel& model)
    {
        this->m_uid = model.m_uid;
        this->m_uuid = model.m_uuid;
        this->m_connectHandler = model.m_connectHandler;
        this->m_connectHandlerInQt = model.m_connectHandlerInQt;
        this->m_managerHandler = model.m_managerHandler;
        return *this;
    }

    virtual int sendData(const char* data, const int size) = 0;
    virtual int reciveData(const char* data, const int size) = 0;



public:
    QString m_uid;
    QString m_uuid;
    ConnectEventHandler* m_connectHandler;
    ConnectEventHandlerInQt* m_connectHandlerInQt;
    ConnectionManage*     m_managerHandler;


};
}

#endif // CONNECTIONMODEL_H
