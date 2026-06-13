// Implementation of common utility functions for logging and network operations
#include <QDebug>
#include <QDir>
#include <QByteArray>
#include <QProcess>
#include <QStringList>
#include <QCoreApplication>
#include <QSettings>
#include <QtNetwork>
#include <QHostInfo>
#include "commonutils.h"


QString CommonUtils::m_logPath = DEFAULT_LOG_FILE_PATH;

void outputMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QString typeText;

    switch(type)
    {
    case QtDebugMsg:
        typeText = QString("Debug:");
        break;
    case QtWarningMsg:
        typeText = QString("Warning:");
        break;
    case QtCriticalMsg:
        typeText = QString("Critical:");
        break;
    case QtFatalMsg:
        typeText = QString("Fatal:");
    }
    QString filePath(context.file);

    static QMutex mutex;

    static bool logToFile = QSettings("setting.ini",QSettings::IniFormat).value("Log/logToFile", true).toBool();
    if(logToFile)
    {
        mutex.lock();
        QFile file(CommonUtils::m_logPath);
        file.open(QIODevice::WriteOnly | QIODevice::Append);
        QTextStream text_stream(&file);
        text_stream << QString("%1 %2 %3 %4 %5").arg(typeText).arg(QLocale("en_US").toString(QDateTime::currentDateTime(),"yyyy-MM-dd hh:mm:ss ddd")).arg(filePath.mid(filePath.lastIndexOf('\\')+1)).arg(context.line).arg(msg);
        file.close();
        mutex.unlock();
    }
}

QString getLogFilePath()
{
    QString logDir = QStandardPaths::standardLocations(QStandardPaths::GenericDataLocation)[0] + "/PICOBusinessSuitData/log";

    if(!QDir(logDir).exists())
    {
        qDebug() << logDir << " not exist!" << Qt::endl;

        if(!QDir().mkpath(logDir))
        {
            qDebug() << logDir << " creat failed!" << Qt::endl;
            return DEFAULT_LOG_FILE_PATH;
        }
        else
        {
            qDebug() << logDir << " creat success!" << Qt::endl;
        }
    }

    QDir sDir(logDir);
    QFileInfoList fileList = sDir.entryInfoList(QDir::Files, QDir::Time);

    while(fileList.size() - MAX_LOG_FILE_NUM > 0)
    {
        if(!QFile::remove(fileList.back().filePath()))
        {
            qDebug() << "remove file error:" << fileList.back().filePath() <<Qt::endl;
        }
        fileList.pop_back();
    }

   QString name = QDateTime::currentDateTime().date().toString("yyyyMMdd") + ".txt";
   if(fileList.size() > 0 && name == fileList.front().fileName())
   {
       return fileList.front().filePath();
   }
   else
   {
       if(fileList.size() >= MAX_LOG_FILE_NUM)
       {
            if(!QFile::remove(fileList.back().filePath()))
            {
               qDebug() << "remove file error:" << fileList.back().filePath() <<Qt::endl;
            }
            fileList.pop_back();
       }

       QFile file(logDir + "/" + name);
       if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
       {
           qDebug() << "creat file failed: " << logDir + "/" + name << Qt::endl;
           return DEFAULT_LOG_FILE_PATH;
       }
       file.close();
   }

   return logDir + "/" + name;
}


CommonUtils::CommonUtils()
{
}

CommonUtils::~CommonUtils()
{}

void CommonUtils::installLogHandler()
{
    m_logPath = getLogFilePath();
    qInstallMessageHandler(outputMessage);
}

QStringList CommonUtils::getLocalIPv4AddressList()
{
    QStringList list;
#ifdef WINDOWS_x86
    QString localHostName = QHostInfo::localHostName();
    QHostInfo info = QHostInfo::fromName(localHostName);
    foreach(QHostAddress address, info.addresses())
    {
        if(address.protocol() == QAbstractSocket::IPv4Protocol)
            list.append(address.toString());
    }
    return list;
#endif
#if defined(LINUX_x86) || defined(LINUX_aarch64)
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    foreach(QNetworkInterface inter, interfaces)
    {
        qDebug() << inter.humanReadableName() << Qt::endl;
        if(inter.humanReadableName().indexOf("LOOPBACK", 0, Qt::CaseInsensitive) != -1
            || inter.humanReadableName().indexOf("lo", 0, Qt::CaseInsensitive) != -1)
        {
            continue;
        }
        else
        {
            QList<QNetworkAddressEntry> entryList = inter.addressEntries();
            foreach(QNetworkAddressEntry entry, entryList)
            {
                if(entry.ip().protocol() == QAbstractSocket::IPv4Protocol)
                {
                    list.append(entry.ip().toString());
                }
            }

        }
    }
    return list;
#endif
}

QString CommonUtils::getLocalIPv4Address()
{
#ifdef WINDOWS_x86
    QString localHostName = QHostInfo::localHostName();
    QHostInfo info = QHostInfo::fromName(localHostName);
    foreach(QHostAddress address, info.addresses())
    {
        if(address.protocol() == QAbstractSocket::IPv4Protocol)
            return address.toString();
    }
    qWarning() << "get local ipv4 failed!" << Qt::endl;
    return "";
#endif
#if defined(LINUX_x86) || defined(LINUX_aarch64)
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    foreach(QNetworkInterface inter, interfaces)
    {
        qDebug() << inter.humanReadableName() << Qt::endl;
        if(inter.humanReadableName().indexOf("LOOPBACK", 0, Qt::CaseInsensitive) != -1
                || inter.humanReadableName().indexOf("lo", 0, Qt::CaseInsensitive) != -1)
        {
            continue;
        }
        else
        {
            QList<QNetworkAddressEntry> entryList = inter.addressEntries();
            foreach(QNetworkAddressEntry entry, entryList)
            {
                if(entry.ip().protocol() == QAbstractSocket::IPv4Protocol)
                {
                    return entry.ip().toString();
                }
            }

        }
    }
    return "";
#endif
}

bool CommonUtils::isOnline()
{
    // wuji-hand-teleop patch (2026-06-02): QNetworkInformation::loadDefaultBackend()
    // was introduced in Qt 6.4. Ubuntu 22.04 ships Qt 6.2.4, where the API does
    // not exist and the upstream code fails to compile. isOnline() is not called
    // anywhere in the codebase, so we keep the Qt 6.4+ path verbatim and stub the
    // older one to return true (assume online).
#if QT_VERSION >= QT_VERSION_CHECK(6, 4, 0)
    if(!QNetworkInformation::loadDefaultBackend())
    {
        qDebug() << "QNetworkInformation load failed." << Qt::endl;
        return false;
    }

    QNetworkInformation::Reachability status = QNetworkInformation::instance()->reachability();
    if(status == QNetworkInformation::Reachability::Local
        || status ==  QNetworkInformation::Reachability::Site
        || status == QNetworkInformation::Reachability::Online)
    {
        return true;
    }
    else
    {
        return false;
    }
#else
    return true;
#endif
}
