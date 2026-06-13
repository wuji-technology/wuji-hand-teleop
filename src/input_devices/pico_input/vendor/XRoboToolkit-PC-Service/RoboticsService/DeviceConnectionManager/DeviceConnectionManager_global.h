// Global definitions and structures for DeviceConnectionManager
#ifndef DEVICECONNECTIONSDK_GLOBAL_H
#define DEVICECONNECTIONSDK_GLOBAL_H

#include <QtCore/qglobal.h>
#include <QByteArray>
#include <QDateTime>
#include <QtDebug>

#if defined(DEVICECONNECTIONSDK_LIBRARY)
#  define DEVICECONNECTIONSDK_EXPORT Q_DECL_EXPORT
#else
#  define DEVICECONNECTIONSDK_EXPORT Q_DECL_IMPORT
#endif


#ifdef WIN32
#define DevConnSDK_API extern "C" __declspec(dllexport)
#elif __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_MAC && !(TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR)
#define DevConnSDK_API __attribute__((__visibility__("default"))) extern "C"
#else
#define DevConnSDK_API __attribute__((__visibility__("default")))
#endif
#else
#define DevConnSDK_API __attribute__((__visibility__("default")))
#endif

namespace DevConnSDK
{
#pragma pack(1)
enum ConnectionState
{
    ConnectionStateDisconnected  = 0,
    ConnectionStateConnected
};

enum SDKError
{
    NONE_ERROR = 0x00,
    SDK_NOT_INIT,
    INVALID_PARAM,
    UNKNOWN_CONNECT_TYPE,
    CALLBACK_SETED_BEFORE,
    QSTYLE_CALLBACK_SETED_BEFORE,
    NORMAL_SCREEN_SHARE_FAILED,
    VR_SCREEN_SHARE_FAILED,
    NOT_BYTERTC_ENGINE,
    NOT_TCPUDP_ENGINE,
    TCPENGINE_SERVER_LISTEN_ERROR,
    TCPENGINE_SERVER_ERROR,
    TCPENGINE_SET_EVENT_HANDLER_BEFORE_INIT,
    PLATFORM_NOT_SUPOORT
};

enum ScreenCaptureSourceType
{
    /**
     * @brief 类型未知
     */
    ScreenCaptureSourceTypeUnknown,

    /**
     * @brief 应用程序的窗口
     */
    ScreenCaptureSourceTypeNormalWindow,

    /**
     * @brief 桌面
     */
    ScreenCaptureSourceTypeScreen,

    /**
     * @brief VR share window
     */
    ScreenCaptureSourceTypeVRWindow,
    /**
     * @brief Steam3D window
     */
    ScreenCaptureSourceTypeSteam3DWindow
};

enum WanWlanMode
{
    WlanMode = 0,
    WanMode
};
enum StreamType
{
    StreamType_Audio = 0,
    StreamType_Video
};
enum ScreenShareType
{
    ScreenShareType_2D,
    ScreenShareType_VR,
    ScreenShareType_3D,
};



struct ByteRtc_InitParam
{
    int  rtcMode;
    int  wlanSearchType;
    bool isOverseas;
};

struct ByteRtc_StartScreenShareParam
{
    int screenIndex;
    ScreenShareType screenShareType;
    int vrPid;
};

struct ByteRtc_StopScreenShareParam
{
    ScreenShareType screenShareType;
};

struct ByteRtc_StartCameraShareParam
{

};

struct ByteRtc_StopCameraShareParam
{

};

struct ByteRtc_StartMonitorParam
{
    const char* uid;
};


struct ByteRtc_StopMonitorParam
{
    const char* uid;
};

struct ByteRtc_StartAudioShareParam
{
    int voiceType;
};

struct ByteRtc_StopAudioShareParam
{
    int voiceType;
};

struct ByteRtc_ScreenShareConfigParam
{
    enum ConfigType
    {
        Resolution = 0,
        FrameRate,
        BitRate,
        VoiceType
    };

    ConfigType type;
    int width;
    int height;
    int fps;
    int bitRate;
    int voiceType;
};

struct ByteRtc_CameraShareConfigParam
{

};

struct ByteRtc_MonitorConfigParam
{

};

struct ByteRtc_AudioShareConfigParam
{
    int voiceType;
};
#pragma pack()
}

#endif // DEVICECONNECTIONSDK_GLOBAL_H
