#include <thread>
#include <QDebug>
#include <QMutex>
#include <QSettings>
#include "PXREAGRPCServer.h"
void PXREAServerAPI::init(quint64 devMng, bool login, bool runAsService)
{
    QSettings settings("setting.ini", QSettings::IniFormat);
    QString addr = settings.value("Service/listenAddr", "127.0.0.1").toString();
    QString port = settings.value("Service/listenPort", "60061").toString();
    QString listenStr= addr+":"+port;
    qDebug() <<"create grpc server:"<<listenStr;
    m_addrPort = listenStr.toUtf8().constData();
    m_devMng = (DeviceManagement*)devMng;
    startService();
    connect(&m_devMng->m_connectEvent, &DevConnSDK::ConnectEventHandlerInQt::userJoinedSignal, this, &PXREAServerAPI::replyDeviceFind);
    connect(&m_devMng->m_connectEvent, &DevConnSDK::ConnectEventHandlerInQt::userLeaveSignal, this, &PXREAServerAPI::replyDeviceMissing);
    connect(m_devMng,&DeviceManagement::replyExistDevice,this,&PXREAServerAPI::replyDeviceFind);
    connect(m_devMng,&DeviceManagement::recvDeviceMessageSignal,this,&PXREAServerAPI::replyDeviceMessage);
}

void PXREAServerAPI::deinit()
{
    stopService();
    qDebug() <<"stop grpc server";
}



void PXREAServerAPI::startService()
{
    if(!m_serviceRunning)
    {
        qDebug() <<"start grpc service";
        Start();
        m_serviceRunning = true;

    }
}

void PXREAServerAPI::stopService()
{
    if(m_serviceRunning)
    {
        qDebug() <<"stop grpc service";
        Stop();
        m_serviceRunning = false;
    }
}


grpc::Status PXREAServerAPI::DeviceControlJson(grpc::ServerContext *context, const PXREAService::DeviceControlParameterJson *request, google::protobuf::Empty *response)
{
    QJsonDocument jsonDoc = QJsonDocument::fromJson(QString::fromStdString(request->parameter()).toUtf8());
    QJsonObject jsonObj = jsonDoc.object();
    m_devMng->sendDeviceControlJson(request->devid().c_str(),request->parameter().c_str());
    qDebug() <<"set device control json"<<request->devid().c_str()<<request->parameter().c_str()<< Qt::endl;
    return grpc::Status::OK;
}

grpc::Status PXREAServerAPI::WatchServerFeedback(grpc::ServerContext *context, const PXREAService::VRPid *request, ::grpc::ServerWriter<PXREAService::ServerFeedback> *writer){
    qDebug() <<"server start feedback of pid"<<request->pid()<< Qt::endl;
    QMutexLocker ml(&m_mtx);
    if(m_allClientFeedback.size() > 0)
    {
        for(auto iter = m_allClientFeedback.begin();iter != m_allClientFeedback.end();++iter)
        {
            qDebug() <<"stop exist pid feedback"<<iter.key()<< Qt::endl;
            iter.value()->StopFeedback();
        }
        m_allClientFeedback.clear();
    }

    auto feedback = std::make_shared<FeedbackController>(writer);


    connect(this,&PXREAServerAPI::replyDeviceFind,feedback.get(),&FeedbackController::replyDeviceFind);
    connect(this,&PXREAServerAPI::replyDeviceMissing,feedback.get(),&FeedbackController::replyDeviceMissing);
    connect(this,&PXREAServerAPI::replyDeviceMessage,feedback.get(),&FeedbackController::replyDeviceMessage);


    m_allClientFeedback[request->pid()] = feedback;
    ml.unlock();
    sdkClientAdded(request->pid());
    qDebug() <<"add pid "<<request->pid()<< Qt::endl;
    feedback->StartFeedbackThread();
    m_devMng->ReplyOnlineDevice();

    feedback->WaitForFeedbackFinish();

    sdkClientRemoved(request->pid());
    qDebug() <<"remove pid "<<request->pid()<< Qt::endl;
    ml.relock();
    m_allClientFeedback.remove(request->pid());
    ml.unlock();

    return grpc::Status::OK;
}

grpc::Status PXREAServerAPI::CancelServerFeedback(grpc::ServerContext *context, const PXREAService::VRPid *request, ::google::protobuf::Empty *response){
    qDebug() <<"recv cancel server feedback command"<<request->pid()<< Qt::endl;
    QMutexLocker ml(&m_mtx);
    int pid = request->pid();
    if(m_allClientFeedback.contains(pid))
    {
        m_allClientFeedback[pid]->StopFeedback();
        m_allClientFeedback.remove(pid);
    }
    else
    {
        qDebug() <<"invalid pid"<<pid<< Qt::endl;
    }
    return grpc::Status::OK;
}


grpc::Status PXREAServerAPI::SendBytesToDevice(grpc::ServerContext *context, const PXREAService::DeviceBytesInfo *request, google::protobuf::Empty *response)
{
    m_devMng->sendBytesToDevice(request->devid().c_str(),QByteArray(request->content().data(),request->content().size()));
    qDebug() <<"send device"<<request->devid().c_str()<<"bytes length"<<request->content().size()<< Qt::endl;
    return grpc::Status::OK;
}

grpc::Status PXREAServerAPI::SendBytesToRoom(grpc::ServerContext *context, const PXREAService::RoomBytesInfo *request, google::protobuf::Empty *response)
{

    m_devMng->sendBytesToRoom(QByteArray(request->content().data(),request->content().size()));
    qDebug() <<"send room bytes length"<<request->content().size()<< Qt::endl;
    return grpc::Status::OK;
}


void PXREAServerAPI::Start()
{
    m_serverThread = std::thread([this]{
        grpc::EnableDefaultHealthCheckService(true);
        grpc::reflection::InitProtoReflectionServerBuilderPlugin();
        grpc::ServerBuilder srvBuilder;
        srvBuilder.AddListeningPort(m_addrPort,grpc::InsecureServerCredentials());
        srvBuilder.RegisterService(this);
        m_server = srvBuilder.BuildAndStart();
        if(m_server)
        {
            m_server->Wait();
        }
        else
        {
            qDebug() <<"start grpc server failed"<< Qt::endl;
        }
    });
}

grpc::Status PXREAServerAPI::SendBeat(grpc::ServerContext *context, const::google::protobuf::Empty *request, ::google::protobuf::Empty *response){
    return grpc::Status::OK;
}


void PXREAServerAPI::Stop()
{
    QMutexLocker ml(&m_mtx);
    for(auto& feedback:m_allClientFeedback)
    {
        feedback->StopFeedback();
    }
    m_allClientFeedback.clear();
    ml.unlock();
    if(m_server)
    {
        m_server->Shutdown();
    }
    m_serverThread.join();
}
