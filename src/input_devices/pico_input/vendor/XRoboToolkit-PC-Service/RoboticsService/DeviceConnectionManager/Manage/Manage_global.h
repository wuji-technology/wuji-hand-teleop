// Global definitions and structures for connection management
#ifndef MANAGE_GLOBAL_H
#define MANAGE_GLOBAL_H

#include <QtCore/qglobal.h>
#include <QByteArray>
#include <QDateTime>
#include <QtDebug>

namespace DevConnSDK
{

#define DEFAULT_TCP_PORT 63901
//udp
#define BROADCAST_UDP_PORT 29888
#define BROADCAST_MEADIA_SERVER_INTERVAL 5000 //ms

#define TCP_CONNECTION_CHECK_INTERVAL 20000 //ms

#define UDP_PACKET_CMD_TCPIP   0x7E

#define TCP_SERVER_HEAD_CODE   0xCF
#define TCP_CLIENT_MSG_HEAD_CODE 0x3F
#define TCP_CLIENT_MSG_TAIL_CODE 0xA5


#define TCP_CLIENT_MSG_CONNECT        0x19
#define TCP_CLIENT_MSG_BATTERY        0x1A
#define TCP_CLIENT_MSG_SENSOR         0x1B
#define TCP_CLIENT_MSG_MONITOR_START  0x2E
#define TCP_CLIENT_MSG_HEART_BEAT     0x23

#pragma pack(1)
struct TCPMsgHead
{
    TCPMsgHead& operator=(const TCPMsgHead& model)
    {
        this->head = model.head;
        this->cmd = model.cmd;
        this->length = model.length;
        return *this;
    }

    unsigned char head;
    unsigned char cmd;
    uint32_t length;
};

struct TCPMsgTail
{
    TCPMsgTail& operator=(const TCPMsgTail& model)
    {
        this->timeStamp = model.timeStamp;
        this->tail = model.tail;
        return *this;
    }

    uint64_t timeStamp;
    unsigned char tail;
};

class TcpMessage
{
public:
    QByteArray m_msg;
    TCPMsgHead m_head;
    TCPMsgTail m_tail;
    TcpMessage()
    {}
    TcpMessage(unsigned char type, const QString& data)
    {
        SetCmd(type,data);
    }
    uint32_t size() const
    {
        return m_msg.size() + sizeof(TCPMsgHead) + sizeof(TCPMsgTail);
    }
    void SetCmd(unsigned char type, const QString& data)
    {
        m_head.cmd = type;
        m_head.length = data.toUtf8().length();
        m_head.head = TCP_SERVER_HEAD_CODE;
        m_msg = data.toUtf8();
        m_tail.timeStamp = QDateTime::currentSecsSinceEpoch();
        m_tail.tail = TCP_CLIENT_MSG_TAIL_CODE;
    }
    QByteArray toByteArray() const
    {
        QByteArray buffer;
        buffer.append(reinterpret_cast<const char *>((&m_head)), sizeof (TCPMsgHead));
        buffer.append(m_msg);
        buffer.append(reinterpret_cast<const char *>((&m_tail)), sizeof (m_tail));
        return buffer;
    };
    template<class T>
    static void parseByteArray(int size, const char* data, const T& handler)
    {
        while(size > 0)
        {
            if(data[0] != TCP_CLIENT_MSG_HEAD_CODE)
            {
                data++;
                size -= 1;
                continue;
            }
            else{}

            TcpMessage tcpMsg;
            TCPMsgHead msgHead;
            TCPMsgTail msgTail;
            if(sizeof (msgHead) > static_cast<unsigned int>(size))
            {
                qDebug() << "data size less than head:" << QString::number(size) << Qt::endl;
                return;
            }
            memcpy(&msgHead, data, sizeof (msgHead));
            data += sizeof (msgHead);
            size -= sizeof (msgHead);
            tcpMsg.m_head = msgHead;
            if(sizeof (TCPMsgTail) + msgHead.length > static_cast<unsigned int>(size))
            {
                qDebug() << "data size less than msg and tail:" + QString::number(size) << Qt::endl;
                return;
            }
            tcpMsg.m_msg.append((const char *)(data), msgHead.length);
            data += msgHead.length;
            size -= msgHead.length;
            memcpy(&msgTail, data, sizeof (TCPMsgTail));
            data += sizeof (msgTail);
            tcpMsg.m_tail = msgTail;
            size -= sizeof (msgTail);
            if (msgTail.tail != TCP_CLIENT_MSG_TAIL_CODE)
            {
                qDebug() << "msg tail code error:" + QString::number(msgTail.tail) << Qt::endl;
                continue;
            }
            handler(tcpMsg);
        }
    }
};

#pragma pack()
}

#endif // MANAGE_GLOBAL_H
