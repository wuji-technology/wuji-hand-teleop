// Qt-style event handler interface for device connection events
#ifndef CONNECTEVENTHANDLERINQT_H
#define CONNECTEVENTHANDLERINQT_H

#include <QObject>
#include <QVector>
#include "../DeviceConnectionManager_global.h"

namespace DevConnSDK
{
class DEVICECONNECTIONSDK_EXPORT ConnectEventHandlerInQt:public QObject
{
    Q_OBJECT
public:
    ConnectEventHandlerInQt();

signals:
    void userJoinedSignal(const QString& uid);
    void userLeaveSignal(const QString& uid);
    void userBinaryMsgSignal(QString uid, int size, QByteArray byteArray);
    void connectStatusSignal(bool flag);
    void errorSignal(int);

};
}
#endif // CONNECTEVENTHANDLERINQT_H
