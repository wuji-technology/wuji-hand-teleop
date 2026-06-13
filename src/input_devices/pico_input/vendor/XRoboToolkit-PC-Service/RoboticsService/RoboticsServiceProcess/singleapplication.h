//Header file for single instance application management

#ifndef SINGLEAPPLICATION_H
#define SINGLEAPPLICATION_H

#include <QCoreApplication>
#include <QLocalServer>
#include <QLocalSocket>
#include <QObject>
#include <QLockFile>

#define SINGLE_APP_NAME "RoboticsServiceProcess_Single_Name"
#define TRY_LOCK_TIMEOUT 1
#define CONNECT_TIMEOUT 3000

class SingleApplication:public QCoreApplication
{
    Q_OBJECT
public:
    SingleApplication(int &argc, char **argv);
    ~SingleApplication();

    void release();

    bool isStartUp();
    void initLocalServer();
    bool connectLocalServer();

public slots:
    void onNewConnection();

signals:
   void activeWindowSiganl();


private:
    QLocalServer m_localServer;
    QLocalSocket m_localSocket;
    QLockFile*   m_instanceLock;

};

#endif // SINGLEAPPLICATION_H
