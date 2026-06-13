//Implementation of single instance application management

#include <QStandardPaths>
#include "singleapplication.h"


SingleApplication::SingleApplication(int &argc, char **argv):QCoreApplication(argc,argv)
{
    m_instanceLock = nullptr;
    connect(&m_localServer, &QLocalServer::newConnection, this, &SingleApplication::onNewConnection);
}

SingleApplication::~SingleApplication()
{
    if(m_instanceLock)
    {
        m_instanceLock->unlock();
        delete m_instanceLock;
        m_instanceLock = nullptr;
    }

}

void SingleApplication::onNewConnection()
{
    emit activeWindowSiganl();
}

bool SingleApplication::isStartUp()
{
    if(m_instanceLock)
    {
        if(!m_instanceLock->tryLock(TRY_LOCK_TIMEOUT))
        {
            qDebug() << "error: " << QString::number(m_instanceLock->error()) << Qt::endl;
            qDebug() << "client has been activate!" << Qt::endl;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        QStringList pathList = QStandardPaths::standardLocations(QStandardPaths::GenericDataLocation);
        QString lockFilePath  = pathList[0] + "/PICOBusinessSuitData/lockfile";
        m_instanceLock = new QLockFile(lockFilePath);

        if(!m_instanceLock->tryLock(1))
        {
            qDebug() << "error: " << QString::number(m_instanceLock->error()) << Qt::endl;
            qDebug() << "client has been activate!" << Qt::endl;
            return true;
        }
        else
        {
            return false;
        }

    }
}

void SingleApplication::initLocalServer()
{
    m_localServer.setSocketOptions(QLocalServer::WorldAccessOption);
    if(!m_localServer.listen(SINGLE_APP_NAME))
    {
        if(m_localServer.serverError() == QAbstractSocket::AddressInUseError)
        {
            QLocalServer::removeServer(SINGLE_APP_NAME);
            if(!m_localServer.listen(SINGLE_APP_NAME))
            {
                qDebug() << "local server listen error: " << m_localServer.errorString() << Qt::endl;
            }
        }
        else
        {
             qDebug() << "local server listen error: " << m_localServer.errorString() << Qt::endl;
        }
    }
}

bool SingleApplication::connectLocalServer()
{
    m_localSocket.connectToServer(SINGLE_APP_NAME);
    if(m_localSocket.waitForConnected(CONNECT_TIMEOUT))
    {
        qDebug() << "connnect to " << SINGLE_APP_NAME << " ok." << Qt::endl;
        return true;
    }
    else
    {

        qDebug() << "connnect to " << SINGLE_APP_NAME << "error: " << m_localSocket.error() << Qt::endl;
        return false;
    }
}

void SingleApplication::release()
{
    if(m_instanceLock)
    {
        m_instanceLock->unlock();
        delete m_instanceLock;
        m_instanceLock = nullptr;
    }
}
