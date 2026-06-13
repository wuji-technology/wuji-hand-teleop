#include "FeedbackController.h"
#include "PXREAGRPCServer.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QUuid>
#include "../Business/DeviceManage/devicemanagement_msg.h"

FeedbackController::FeedbackController(::grpc::ServerWriter<PXREAService::ServerFeedback> *writer){

    m_writer = writer;
    qDebug() <<"create controller feedback"<< Qt::endl;
    m_waitTimes.store(0);
    this->moveToThread(&m_thd);

    QObject::connect(this,&FeedbackController::replyDeviceFind,this,[this](const QString& devid){
        qDebug() <<"reply device find"<<devid<< Qt::endl;

        PXREAService::ServerFeedback feedBackMsg;
        feedBackMsg.set_name("deviceFind");
        feedBackMsg.set_devid(devid.toUtf8().constData());
        m_writer->Write(feedBackMsg);
    });
    QObject::connect(this,&FeedbackController::replyDeviceMissing,this,[this](const QString& devid){
        qDebug() <<"reply device missing"<<devid<< Qt::endl;
        PXREAService::ServerFeedback feedBackMsg;
        feedBackMsg.set_name("deviceMissing");
        feedBackMsg.set_devid(devid.toUtf8().constData());
        m_writer->Write(feedBackMsg);
    });

    QObject::connect(this,&FeedbackController::replyDeviceMessage,this,[this](QString devid, int type, QByteArray msgbody){onReplyDeviceMessage(devid,type,msgbody);});

}

FeedbackController::~FeedbackController()
{

    qDebug() <<"destroy feedback Controller ... "<< Qt::endl;
    while(m_waitTimes.load() > 0)
    {
        QThread::msleep(100);
    }
    qDebug() <<"destroy feedback Controller done "<< Qt::endl;
}

void FeedbackController::StartFeedbackThread(){

    m_thd.start();
    qDebug() <<"start feedback Qthread"<< Qt::endl;
}

void FeedbackController::WaitForFeedbackFinish()
{
    ++m_waitTimes;
    m_thd.wait();
    --m_waitTimes;
    qDebug() <<"detect feedback QThread exit"<< Qt::endl;
}

void FeedbackController::StopFeedback()
{
    qDebug() <<"quit feedback Qthread"<< Qt::endl;
    m_thd.quit();
    ++m_waitTimes;
    m_thd.wait();
    --m_waitTimes;
    qDebug() <<"detect feedback QThread exit"<< Qt::endl;
}



void FeedbackController::onReplyDeviceMessage(QString devid, int type, QByteArray msgbody)
{
    PXREAService::ServerFeedback feedBackMsg;
    switch (type)
    {
    case PXREAServerMsgBattery:
    {
        feedBackMsg.set_name("deviceBattery");
        auto btry = new PXREAService::DeviceBattery();
        btry->set_devid(devid.toUtf8().constData());
        btry->set_battery(QString::fromUtf8(msgbody).toUInt());
        feedBackMsg.set_allocated_devbattery(btry);
    }
    break;
    case PXREAServerMsgLock:
    {
        feedBackMsg.set_name("deviceLock");
        auto lkRes = new PXREAService::DeviceStatus();
        lkRes->set_devid(devid.toUtf8().constData());
        QJsonParseError e;
        auto resObj = QJsonDocument::fromJson(msgbody, &e).object();
        lkRes->set_status(resObj.value("result").toInt());
        feedBackMsg.set_allocated_devstatus(lkRes);
    }
    break;
    case PXREAServerMsgUnlock:
    {
        feedBackMsg.set_name("deviceUnlock");
        auto lkRes = new PXREAService::DeviceStatus();
        lkRes->set_devid(devid.toUtf8().constData());
        lkRes->set_status(QString::fromUtf8(msgbody).toInt());
        feedBackMsg.set_allocated_devstatus(lkRes);
    }
    break;
    case PXREAServerMsgConnect:
    {
        feedBackMsg.set_name("deviceConnect");
        auto sts = new PXREAService::DeviceStatus();
        sts->set_devid(devid.toUtf8().constData());
        sts->set_status(QString::fromUtf8(msgbody).toInt());
        feedBackMsg.set_allocated_devstatus(sts);
    }
    break;
    case PXREAServerMsgSensor:
    {
        feedBackMsg.set_name("deviceSensor");
        auto sts = new PXREAService::DeviceStatus();
        sts->set_devid(devid.toUtf8().constData());
        sts->set_status(QString::fromUtf8(msgbody).toInt());
        feedBackMsg.set_allocated_devstatus(sts);
    }
    break;
    case PXREAServerMsgShutdown:
    {
        feedBackMsg.set_name("deviceShutdown");
        feedBackMsg.set_devid(devid.toUtf8().constData());
    }
    break;
    case PXREAServerMsgResume:
    {
        feedBackMsg.set_name("deviceResume");
        feedBackMsg.set_devid(devid.toUtf8().constData());
    }
    break;
    case PXREAServerMsgDeviceModel:
    {
        feedBackMsg.set_name("deviceModel");
        auto dm = new PXREAService::DeviceModel();
        dm->set_devid(devid.toUtf8().constData());
        dm->set_model(msgbody.constData());
        feedBackMsg.set_allocated_devmodel(dm);
    }
    break;
    case PXREAServerMsgCurrentApplication:
    {
        feedBackMsg.set_name("CurrentApplication");
        auto ca = new PXREAService::CurrentApplication();
        ca->set_devid(devid.toUtf8().constData());
        ca->set_appname(msgbody.constData());
        feedBackMsg.set_allocated_currentapp(ca);
    }
    break;
    case PXREAServerMsgControllerBattery:
    {
        feedBackMsg.set_name("ControllerBattery");
        auto cb = new PXREAService::ControllerBattery();
        cb->set_devid(devid.toUtf8().constData());
        QString str = QString::fromUtf8(msgbody);
        cb->set_controllerid(str.split('|')[0].toInt());
        cb->set_connected(str.split('|')[1].toUInt());
        cb->set_battery(str.split('|')[2].toUInt());
        feedBackMsg.set_allocated_ctrllerbtry(cb);
    }
    break;
    case PXREAServerMsgScreenState:
    {
        feedBackMsg.set_name("screenState");
        auto srcState = new PXREAService::DeviceStatus();
        srcState->set_devid(devid.toUtf8().constData());
        srcState->set_status(QString::fromUtf8(msgbody).toInt());
        feedBackMsg.set_allocated_devstatus(srcState);
    }
    break;
    case PXREAServerMsgVideoControlResult:
    {
        feedBackMsg.set_name("videoControlResult");
        auto vcr = new PXREAService::VideoControlResult();
        vcr->set_devid(devid.toUtf8().constData());
        QJsonParseError e;
        auto resObj = QJsonDocument::fromJson(msgbody, &e).object();
        vcr->set_action(resObj.value("action").toString().toUtf8().constData());
        vcr->set_result(resObj.value("result").toInt());
        vcr->set_detail(resObj.value("detail").toString().toUtf8().constData());
        vcr->set_errorcode(resObj.value("errorcode").toInt());
        feedBackMsg.set_allocated_videocontrolresult(vcr);
    }
    break;
    case PXREAServerMsgDeviceAlias:
    {
        feedBackMsg.set_name("deviceAlias");
        auto dai = new PXREAService::DeviceAliasInfo();
        dai->set_devid(devid.toUtf8().constData());
        dai->set_alias(msgbody.constData());
        feedBackMsg.set_allocated_devicealiasinfo(dai);
    }
    break;
    case PXREAServerMsgDeviceStateJson:
    {
        feedBackMsg.set_name("deviceStateJson");
        auto dsj = new PXREAService::DeviceStateJson();
        dsj->set_devid(devid.toUtf8().constData());
        dsj->set_statejson(msgbody.constData());
        feedBackMsg.set_allocated_devicestatejson(dsj);
    }
    break;
    case PXREAServerSendCustomMessage:
    {
        feedBackMsg.set_name("deviceCustomMessage");
        auto db = new PXREAService::DeviceBlob();
        db->set_devid(devid.toUtf8().constData());
        // 直接使用二进制数据，不进行任何转换
        db->set_content(msgbody.constData(), msgbody.size());
        feedBackMsg.set_allocated_devblob(db);
    }
    }
    if(feedBackMsg.name().size()>0)
    {
        m_writer->Write(feedBackMsg);
    }
    else
    {
        qDebug() <<"ignore msg type"<<type<< Qt::endl;
    }
}





