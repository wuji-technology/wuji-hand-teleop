// High-precision timer implementation using Qt
#ifndef PRECISETIMER_H
#define PRECISETIMER_H

#include <QTimer>
class PreciseTimer
{
public:
    explicit PreciseTimer()
    {
        m_tm.setTimerType(Qt::PreciseTimer);
    }
    template<class T>
    void Start(int msec,const T& proc)
    {
        QObject::disconnect(&m_tm,&QTimer::timeout,nullptr,nullptr);
        QObject::connect(&m_tm,&QTimer::timeout,proc);
        m_tm.start(msec);
    }
    void Stop()
    {
        m_tm.stop();
    }

private:
    QTimer m_tm;
};

#endif // PRECISETIMER_H
