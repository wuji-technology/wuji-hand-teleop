// Utility functions interface for network operations
#ifndef UTILS_H
#define UTILS_H
#include <QObject>

namespace DevConnSDK
{
class Utils
{
public:
    Utils();
    static QString getLocalIPv4Address();
    static QStringList getLocalIPv4AddressList();
};
}
#endif // UTILS_H
