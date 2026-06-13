//Main entry point for the Enterprise Assistant Client application

#include <QCoreApplication>
#include <QTextCodec>
#include <QSslSocket>
#include <QLockFile>
#include <QProcess>
#include <QElapsedTimer>
#include "commonutils.h"
#include "business.h"
#include "singleapplication.h"
#include "PXREAGRPCServer.h"


#define STR(R) #R
#define STRVALUE(R) STR(R)

// wuji-hand-teleop patch (2026-06-02): upstream had
// `Q_DECLARE_METATYPE(QSharedPointer<QImage>)` here, but QImage is never
// referenced anywhere else in this codebase (the daemon talks pose/JSON,
// not images) and registering the metatype requires QtGui — which this
// headless service does not otherwise need. Drop the dead declaration so
// the build does not need to link Qt::Gui on top of Core/Network/Core5Compat.

enum PlatformType
{
    WINDOWS_X86 = 0,
    LINUX_X86,
    LINUX_AARCH64
};


int main(int argc, char *argv[])
{
    SingleApplication app(argc, argv);

    QElapsedTimer ElapsedTimer;
    ElapsedTimer.start();

#ifdef QT_NO_DEBUG
    qDebug() << "release mode";
    CommonUtils::installLogHandler();
#else
    qDebug() << "debug mode";
    program = "./LoadingPorcessDebug.exe";
#endif

    if(app.isStartUp())
    {
        app.connectLocalServer();
        return 0;
    }
    else
    {
        app.initLocalServer();
    }

    QTextCodec* codec = QTextCodec::codecForName("utf-8");
    QTextCodec::setCodecForLocale(codec);

    app.setOrganizationDomain("picoxr.com");
    app.setApplicationName("RoboticsServiceProcess");

    auto sdk = new PXREAServerAPI(&app);


#ifdef WINDOWS_x86
    quint32 platformType = WINDOWS_X86;
#endif

#ifdef LINUX_x86
    quint32 platformType = LINUX_X86;
#endif

#ifdef LINUX_aarch64
    quint32 platformType = LINUX_AARCH64;
#endif
    DeviceManagement deviceManage;
    Business business;
    business.setDeviceManager(deviceManage.getThisPoint());
    business.init();

    sdk->init(deviceManage.getThisPoint(), true, true);
    qDebug() << "耗时: " <<ElapsedTimer.elapsed() << "毫秒" << Qt::endl;

    app.exec();
    sdk->deinit();
    app.release();


    return 0;
}
