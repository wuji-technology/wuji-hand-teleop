// Device model class for storing and managing device information and states
#ifndef DEVICEMODEL_H
#define DEVICEMODEL_H
#include <QObject>
#include <QAbstractListModel>
#include <QVector>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

#include "Business_global.h"


enum DeviceStatus
{
    DeivceSensorFarAway = 0,
    DeivceSensorNear,
    DeviceOffline
};


enum Device_OnOff_Status
{
    Device_On = 0,
    Device_ShutDown
};

enum SensorType
{
    SensorFarAway = 0,
    SensorNear
};

enum ChargeStatus
{
    OffCharge = 0,
    OnCharge
};


enum BatteryStatus
{
    Battery0Percent = 0,
    Battery20Percent = 20,
    Battery40Percent = 40,
    Battery60Percent = 60,
    Battery80Percent = 80,
    Battery100Percent = 100
};


class DeviceModel:public QObject
{
    Q_OBJECT
public:

    enum DeviceState
    {
        AllState = 0,
        OffLineState,
        FreeState,
        ControlledReadyState,
        ControlledPlayState
    };

    enum SolutionDeployStatus
    {
        NotDeployed = 1,
        WaitForDeploy,
        Deploying,
        Deployed ,
        DeployError
    };

    enum SolutionDeployError
    {
        NoError = 0,
        DeviceSpaceNotEnough,
        DeviceOffline,
        ReourceDownloadFailed,
        UserCancel,
        ControllerDisconnected,
        PCUploadFailed,
        PCSpaceNotEnough,
        PCFileLost,
        InstallSpaceNotEnough,
        NiginxServiceNotInstall,
        NiginxServiceNotStart,
        VRAlreadyLocked,
        UnknownError = 0xFF
    };

    enum DeployingType
    {
        BroadCast = 0,
        AutoPlay = 2,
        NotDeploying = 10001
    };

    DeviceModel()
    {
        this->m_id = "-1";
        this->m_uid = "";
        this->m_sn = "";
        this->m_sonsor = SensorType::SensorFarAway;
        this->m_deviceOnOff = Device_OnOff_Status::Device_ShutDown;
        this->m_batteryStatus = BatteryStatus::Battery100Percent;
        this->m_chargeStatus = ChargeStatus::OffCharge;
        this->m_isPlay = false;
        this->m_isControl = false;
        this->m_isOnline = false;
        this->m_invalidType = VALID;
        this->m_isAdded = false;
        this->m_isSelected = false;
        this->m_isShared = false;
        this->m_tempIndex = "";
        this->m_isLock = false;
        this->m_alias = "";
        this->m_isMyLock = false;
        this->m_solutionID = "";
        this->m_solutionVersion = "";
        this->m_isSelectDeploy = false;
        this->m_deployStatus = NotDeployed;
        this->m_deployError = NoError;
        this->m_deployTime = "";
        this->m_deviceType = "";
        this->m_deployProgress = 0;
        this->m_deployingType = DeployingType::NotDeploying;
        this->m_batteryVal = 0;
        this->m_playID = "";
        this->m_state = DeviceState::OffLineState;
        this->m_caseName = "";
        this->m_deployErrMsg = "";
        this->m_isMonitoring = false;
        this->m_isPiping = false;
        this->m_screenOn = false;
        this->m_volume = 50;
        this->m_isCalling = false;
        this->m_micVol = 0;
        this->m_micMute = false;
        this->m_monitorStartTime = 0;
        this->m_vrVersion = "";
        this->m_cpVer = "";
        this->m_lCtrBattery = -1;
        this->m_rCtrBattery = -1;
    };

    DeviceModel(const DeviceModel& model)
    {
        this->m_id = model.m_id;
        this->m_uid = model.m_uid;
        this->m_sn = model.m_sn;
        this->m_invalidType = model.m_invalidType;
        this->m_isAdded = model.m_isAdded;
        this->m_isSelected = model.m_isSelected;
        this->m_sonsor = model.m_sonsor;
        this->m_isPlay = model.m_isPlay;
        this->m_chargeStatus = model.m_chargeStatus;
        this->m_batteryStatus = model.m_batteryStatus;
        this->m_deviceOnOff = model.m_deviceOnOff;
        this->m_isControl = model.m_isControl;
        this->m_isOnline = model.m_isOnline;
        this->m_isShared = model.m_isShared;
        this->m_tempIndex = model.m_tempIndex;
        this->m_isLock = model.m_isLock;
        this->m_alias = model.m_alias;
        this->m_isMyLock = model.m_isMyLock;
        this->m_solutionID = model.m_solutionID;
        this->m_solutionVersion = model.m_solutionVersion;
        this->m_isSelectDeploy = model.m_isSelectDeploy;
        this->m_deployStatus = model.m_deployStatus;
        this->m_deployError = model.m_deployError;
        this->m_deployTime = model.m_deployTime;
        this->m_deviceType = model.m_deviceType;
        this->m_deployProgress = model.m_deployProgress;
        this->m_deployingType = model.m_deployingType;
        this->m_batteryVal = model.m_batteryVal;
        this->m_playID = model.m_playID;
        this->m_state = model.m_state;
        this->m_caseName = model.m_caseName;
        this->m_deployErrMsg = model.m_deployErrMsg;
        this->m_isMonitoring = model.m_isMonitoring;
        this->m_isPiping = model.m_isPiping;
        this->m_screenOn = model.m_screenOn;
        this->m_volume = model.m_volume;
        this->m_isCalling = model.m_isCalling;
        this->m_micVol = model.m_micVol;
        this->m_micMute = model.m_micMute;
        this->m_monitorStartTime = model.m_monitorStartTime;
        this->m_vrVersion = model.m_vrVersion;
        this->m_cpVer = model.m_cpVer;
        this->m_lCtrBattery = model.m_lCtrBattery;
        this->m_rCtrBattery = model.m_rCtrBattery;
    }

    DeviceModel& operator=(const DeviceModel& model)
    {
        this->m_id = model.m_id;
        this->m_uid = model.m_uid;
        this->m_sn = model.m_sn;
        this->m_invalidType = model.m_invalidType;
        this->m_isAdded = model.m_isAdded;
        this->m_isSelected = model.m_isSelected;
        this->m_sonsor = model.m_sonsor;
        this->m_chargeStatus = model.m_chargeStatus;
        this->m_batteryStatus = model.m_batteryStatus;
        this->m_deviceOnOff = model.m_deviceOnOff;
        this->m_isPlay = model.m_isPlay;
        this->m_isControl = model.m_isControl;
        this->m_isOnline = model.m_isOnline;
        this->m_isShared = model.m_isShared;
        this->m_tempIndex = model.m_tempIndex;
        this->m_isLock = model.m_isLock;
        this->m_alias = model.m_alias;
        this->m_isMyLock = model.m_isMyLock;
        this->m_solutionID = model.m_solutionID;
        this->m_solutionVersion = model.m_solutionVersion;
        this->m_isSelectDeploy = model.m_isSelectDeploy;
        this->m_deployStatus = model.m_deployStatus;
        this->m_deployError = model.m_deployError;
        this->m_deployTime = model.m_deployTime;
        this->m_deviceType = model.m_deviceType;
        this->m_deployProgress = model.m_deployProgress;
        this->m_deployingType = model.m_deployingType;
        this->m_batteryVal = model.m_batteryVal;
        this->m_playID = model.m_playID;
        this->m_state = model.m_state;
        this->m_caseName = model.m_caseName;
        this->m_deployErrMsg = model.m_deployErrMsg;
        this->m_isMonitoring = model.m_isMonitoring;
        this->m_isPiping = model.m_isPiping;
        this->m_screenOn = model.m_screenOn;
        this->m_volume = model.m_volume;
        this->m_isCalling = model.m_isCalling;
        this->m_micVol = model.m_micVol;
        this->m_micMute = model.m_micMute;
        this->m_monitorStartTime = model.m_monitorStartTime;
        this->m_vrVersion = model.m_vrVersion;
        this->m_cpVer = model.m_cpVer;
        this->m_lCtrBattery = model.m_lCtrBattery;
        this->m_rCtrBattery = model.m_rCtrBattery;
        return *this;
    }

   ~DeviceModel(){}


public:
    //invalid type enum
    static const int VALID = 0;  //3.0版本后不再使用
    static const int INVALID = 1; //3.0版本后不再使用

public:
    QString     m_id;
    QString     m_uid;
    QString     m_sn;
    ChargeStatus m_chargeStatus;
    BatteryStatus m_batteryStatus;
    Device_OnOff_Status m_deviceOnOff;
    SensorType  m_sonsor;
    int         m_invalidType; //add invalid type
    bool        m_isAdded;
    bool        m_isSelected; //select device to import
    bool        m_isShared;   //select device in video share
    bool        m_isSelectDeploy; //select device to deploy solution
    bool        m_isOnline;
    bool        m_isControl;
    bool        m_isPlay;

    QString     m_tempIndex;
    bool        m_isLock;
    QString     m_alias;
    bool        m_isMyLock;
    QString     m_solutionID;
    QString     m_solutionVersion;
    int         m_deployProgress;
    SolutionDeployStatus m_deployStatus;
    SolutionDeployError  m_deployError;
    QString     m_deployTime;
    QString     m_deviceType;
    DeployingType m_deployingType;
    int         m_batteryVal;
    QString     m_playID;
    DeviceState m_state;
    QString     m_caseName;
    QString     m_deployErrMsg;
    bool        m_isMonitoring;
    bool        m_isPiping;
    bool        m_screenOn;
    int         m_volume;
    bool        m_isCalling;
    int         m_micVol;
    bool        m_micMute;
    qint64      m_monitorStartTime;
    QString     m_vrVersion;
    QString     m_cpVer;
    int         m_lCtrBattery;
    int         m_rCtrBattery;

};
#endif // DEVICEMODEL_H
