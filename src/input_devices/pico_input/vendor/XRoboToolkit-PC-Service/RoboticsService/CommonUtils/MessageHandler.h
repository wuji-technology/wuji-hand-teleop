// Message handler for logging and debug output
#pragma once
#include <QDebug>
#include <QDateTime>
#include <QMutex>
#include <QFile>
#include <QSettings>
#include <windows.h>
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
        QFile file("log.txt");
        file.open(QIODevice::WriteOnly | QIODevice::Append);
        QTextStream text_stream(&file);
        text_stream << QString("%1 %2 %3 %4 %5").arg(typeText).arg(QLocale("en_US").toString(QDateTime::currentDateTime(),"yyyy-MM-dd hh:mm:ss ddd")).arg(filePath.midRef(filePath.lastIndexOf('\\')+1)).arg(context.line).arg(msg);
        file.close();
        mutex.unlock();
    }
    else
    {
        OutputDebugStringA(QString("%1 %2 %3 %4 %5").arg(typeText).arg(QLocale("en_US").toString(QDateTime::currentDateTime(),"yyyy-MM-dd hh:mm:ss ddd")).arg(filePath.midRef(filePath.lastIndexOf('\\')+1)).arg(context.line).arg(msg).toLocal8Bit().constData());
    }

}
