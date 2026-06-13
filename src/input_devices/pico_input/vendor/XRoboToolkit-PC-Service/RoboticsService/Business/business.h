// Main business logic class for managing application components
#ifndef BUSINESS_H
#define BUSINESS_H

#include <QObject>
#include <QUrl>
#include <QTimer>
#include <QSettings>
#include "Business_global.h"

class SolutionManagement;
class DeviceManagement;
class RenderBase;
class LoginManagement;

class BUSINESSSHARED_EXPORT Business:public QObject
{
    Q_OBJECT

public:
    Business();

signals:
    void deviceManagerSetedSignal();

public:


public slots:
    void init();

    quint64 getThisPoint()
    {
        return (quint64)this;
    }

    void setDeviceManager(quint64 addr);
    void onDeviceManagerSeted();

public:
    DeviceManagement*   m_deviceManagerPtr;

};



#endif // BUSINESS_H
