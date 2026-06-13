// Global definitions and shared structures for business layer
#ifndef BUSINESS_GLOBAL_H
#define BUSINESS_GLOBAL_H

#include <QDebug>
#include <QDateTime>
#include <QtCore/qglobal.h>
#include <QByteArray>
#include <QTextCodec>


#if defined(BUSINESS_LIBRARY)
#  define BUSINESSSHARED_EXPORT Q_DECL_EXPORT
#else
#  define BUSINESSSHARED_EXPORT Q_DECL_IMPORT
#endif

//udp

#define UDP_PACKET_CMD_MEDIAIP 0x7F
#define UDP_PACKET_CMD_TCPIP   0x7E


//server cmd
#define TCP_SERVER_HEAD_CODE                 0xCF
#define TCP_SERVER_CMD_DEVICE_CONTROL_JSON   0x5F
#define TCP_SERVER_CMD_SEND_BYTES_TO_DEVICE  0x71


//clinet msg
#define TCP_CLIENT_MSG_HEAD_CODE            0x3F
#define TCP_CLIENT_MSG_CONNECT              0x19
#define TCP_CLIENT_MSG_DEVICE_STATE_JSON    0x6D
#define TCP_CLIENT_MSG_SEND_BYTES_TO_DEVICE 0x72

//msg tail
#define TCP_CLIENT_MSG_TAIL_CODE 0xA5

#define LOG_INFO(x) (qDebug() << x << Qt::endl);

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
                LOG_INFO("data size less than head:" + QString::number(size));
                return;
            }
            memcpy(&msgHead, data, sizeof (msgHead));
            data += sizeof (msgHead);
            size -= sizeof (msgHead);
            tcpMsg.m_head = msgHead;
            if(sizeof (TCPMsgTail) + msgHead.length > static_cast<unsigned int>(size))
            {
                LOG_INFO("data size less than msg and tail:" + QString::number(size));
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
                LOG_INFO("msg tail code error:" + QString::number(msgTail.tail));
                continue;
            }
            handler(tcpMsg);
        }
    }
};

Q_DECLARE_METATYPE(TcpMessage);
#pragma pack()



#endif // BUSINESS_GLOBAL_H
