// Implementation of main business logic class
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QUrl>
#include <QTextCodec>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSettings>
#include <QStandardPaths>
#include "business.h"

#include "./DeviceManage/devicemanagement.h"


Business::Business()
{
    m_deviceManagerPtr = nullptr;
}

void Business::init()
{
}


void Business::setDeviceManager(quint64 addr)
{
    m_deviceManagerPtr = reinterpret_cast<DeviceManagement*>(addr);
    onDeviceManagerSeted();
}

void Business::onDeviceManagerSeted()
{

    emit deviceManagerSetedSignal();
    if(m_deviceManagerPtr)
    {
        m_deviceManagerPtr->init();
    }
    else
    {
        LOG_INFO("deviceManager is nullptr");
    }
}
