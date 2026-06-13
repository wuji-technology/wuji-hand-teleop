// Message type definitions for device management communication
#ifndef DEVICEMANAGEMENT_MSG_H
#define DEVICEMANAGEMENT_MSG_H

typedef enum
{
    PXREAServerMsgBattery               = 1,
    PXREAServerMsgLock                  = 1<<1,
    PXREAServerMsgUnlock                = 1<<2,
    PXREAServerMsgConnect               = 1<<3,
    PXREAServerMsgSensor                = 1<<4,
    PXREAServerMsgShutdown              = 1<<5,
    PXREAServerMsgResume                = 1<<6,
    PXREAServerMsgMonitorParameter      = 1<<7,
    PXREAServerMsgDeviceModel           = 1<<8,
    PXREAServerMsgCurrentApplication    = 1<<9,
    PXREAServerMsgControllerBattery     = 1<<10,
    PXREAServerMsgScreenState           = 1<<11,
    PXREAServerMsgVideoControlResult    = 1<<12,
    PXREAServerMsgDeviceAlias           = 1<<13,
    PXREAServerMsgDeviceStateJson       = 1<<14,
    PXREAServerSendCustomMessage        = 1<<15
} PXREAServerMsgType;

#endif // DEVICEMANAGEMENT_MSG_H
