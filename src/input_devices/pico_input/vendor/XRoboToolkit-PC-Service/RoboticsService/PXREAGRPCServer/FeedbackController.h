//Controller class for handling server feedback and device messages
#ifndef FEEDBACKCONTROLLER_H
#define FEEDBACKCONTROLLER_H

#include <QObject>
#include <QThread>
#include <QDebug>
#include <PXREAService.pb.h>
#include <PXREAService.grpc.pb.h>
#include <DeviceManage/devicemanagement.h>
#include <Business_global.h>
#include <atomic>
#include <QHash>


class FeedbackController : public QObject
{
    Q_OBJECT
public:
    FeedbackController(::grpc::ServerWriter<PXREAService::ServerFeedback> *writer);
    ~FeedbackController();
public:
    void StartFeedbackThread();
    void WaitForFeedbackFinish();
    void StopFeedback();
signals:
     void startFrameCapture(unsigned width, unsigned height,unsigned fps);
     void stopFrameCapture();
     void replyDeviceFind(QString devid);
     void replyDeviceMissing(QString devid);
     void replyDeviceMessage(QString devid, int type, QByteArray msgbody);
     void replyFirstMonitorScreenFrame(QString devid, int width, int height);
     void replyMonitorScreenFrame(QString devid, QByteArray rtcFrame, int width, int height, int bytesPerLine, int pixFormat);
     void replyFirstMonitorStreamFrame(QString devid, int width, int height);
     void replyMonitorStreamFrame(QString devid, QByteArray rtcFrame, int width, int height, int bytesPerLine, int pixFormat);
     void createScreenMonitorSharedMemory(QString uid);
     void destroyScreenMonitorSharedMemory(QString uid);
     void createStreamMonitorSharedMemory(QString uid);
     void destroyStreamMonitorSharedMemory(QString uid);
public slots:
    void onReplyDeviceMessage(QString devid, int type, QByteArray msgbody);


private:
     QThread m_thd;
     std::atomic_int m_waitTimes;
     ::grpc::ServerWriter<PXREAService::ServerFeedback> *m_writer;
};

#endif // FEEDBACKCONTROLLER_H
