// Implementation of utility functions for network operations
#include <QHostInfo>
#include <QStringList>
#include <QNetworkInterface>
#include "utils.h"

DevConnSDK::Utils::Utils()
{

}

QString DevConnSDK::Utils::getLocalIPv4Address()
{
    QString localHostName = QHostInfo::localHostName();
    QHostInfo info = QHostInfo::fromName(localHostName);

    foreach(QHostAddress address,info.addresses())
    {
        if(address.protocol() == QAbstractSocket::IPv4Protocol)
        {
            return address.toString();
        }
    }

    qWarning() << "get local ipv4 failed!" << Qt::endl;
    return "";
}


QStringList DevConnSDK::Utils::getLocalIPv4AddressList()
{
    QStringList ipList;
#ifdef WINDOWS_x86
    QString localHostName = QHostInfo::localHostName();
    QHostInfo info = QHostInfo::fromName(localHostName);

    foreach(QHostAddress address,info.addresses())
    {
        if(address.protocol() == QAbstractSocket::IPv4Protocol)
        {
            ipList << address.toString();
        }
    }
#endif
#if defined(LINUX_x86) || defined(LINUX_aarch64)
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    foreach(QNetworkInterface inter, interfaces)
    {
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
                    ipList << entry.ip().toString();
                }
            }

        }
    }
#endif
    return ipList;
}
