//Server SDK header file for handling gRPC server implementation
#ifndef PXREASERVERSDK_H
#define PXREASERVERSDK_H

#if defined(PXREASERVERSDK_LIBRARY)
#  define PXREASERVERSDK_EXPORT Q_DECL_EXPORT
#else
#  define PXREASERVERSDK_EXPORT Q_DECL_IMPORT
#endif

#include <QObject>
#include <stdint.h>
#include <QMutex>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <DeviceManage/devicemanagement.h>
#include "PXREAService.grpc.pb.h"
#include "PXREAService.pb.h"
#include "FeedbackController.h"

class PXREASERVERSDK_EXPORT PXREAServerAPI: public QObject,public PXREAService::EAService::Service
{
    Q_OBJECT
public:
    explicit PXREAServerAPI(QObject *parent = nullptr) : QObject(parent),m_serviceRunning(false)
    {
    }
signals:
    void sdkClientAdded(int pid);
    void sdkClientRemoved(int pid);
    void replyDeviceFind(QString devid);
    void replyDeviceMissing(QString devid);
    void replyDeviceMessage(QString devid,int type,QByteArray msgbody);


public slots:
    void init(quint64 devMng, bool login, bool runAsService);
    void deinit();
    void startService();
    void stopService();

    quint64 getThisPoint()
    {
        return (quint64)this;
    }

public:
    virtual grpc::Status SendBytesToDevice(grpc::ServerContext *context, const PXREAService::DeviceBytesInfo *request, google::protobuf::Empty *response) override;
    virtual grpc::Status SendBytesToRoom(grpc::ServerContext *context, const PXREAService::RoomBytesInfo *request, google::protobuf::Empty *response) override;
    virtual grpc::Status DeviceControlJson(grpc::ServerContext *context, const PXREAService::DeviceControlParameterJson *request, ::google::protobuf::Empty *response) override;
    virtual grpc::Status SendBeat(grpc::ServerContext *context, const::google::protobuf::Empty *request, ::google::protobuf::Empty *response) override;
    virtual grpc::Status WatchServerFeedback(grpc::ServerContext *context, const PXREAService::VRPid *request, ::grpc::ServerWriter<PXREAService::ServerFeedback> *writer) override;
    virtual grpc::Status CancelServerFeedback(grpc::ServerContext *context, const PXREAService::VRPid *request, ::google::protobuf::Empty *response) override;
    void Start();
    void Stop();
private:
    bool m_serviceRunning;
    std::thread m_serverThread;
    std::unique_ptr<grpc::Server> m_server;
    std::string m_addrPort;
    QHash<int,std::shared_ptr<FeedbackController>> m_allClientFeedback;
    QMutex m_mtx;
    DeviceManagement* m_devMng;
    QSet<QString> m_screenMonitorDevices;
    QSet<QString> m_streamMonitorDevices;

};



#endif // PXREASERVERSDK_H
