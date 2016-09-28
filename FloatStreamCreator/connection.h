
#ifndef CONNECTION_H_
#define CONNECTION_H_

#include <QtSerialPort/QSerialPort>
#include <QVector>
#include <QThread>
#include <QMutex>

#include "packet.h"


enum CommStatus
{
    CS_Initializing,
    CS_NoDevice,
    CS_NoElev8,
    CS_Connected,
};


class Connection : public QThread
{
    Q_OBJECT  // only need this if adding slots

signals:
    void connectionMade();

public:
    Connection(QObject *parent = 0);
    ~Connection();   

    void StartConnection(void);
    void StopConnection(void);

    void run();

    void setActive(bool val)  { active = val;}
    bool Active(void) const   {return active;}

    void setConnected(bool val)  {connected = val;}
    bool Connected(void) const   {return connected;}

	void Reset(void);

    CommStatus Status(void) const {return commStat;}

    void Send( quint8 * bytes , int count );

    packet * GetPacket(void);


protected:
    quint16 Checksum( quint16 checksum, const quint8 * buf, int Length );
    quint16 Checksum( quint16 checksum, quint16 val );

    void Update(void);
    void ProcessByte( quint8 b );
    void AttemptConnect(void);
    void Disconnect(void);


private:
    QSerialPort * serial;
    QMutex mutex;

    volatile bool quit;
    volatile bool connected;
    volatile bool active;

    CommStatus commStat;

    int sigByteIndex;
    int packetByteIndex;

    static const int packetCount = 128;
    packet * packetsArray[packetCount];
    int head, tail;

    QByteArray toSend;

    packet * currentPacket;
    quint16 currentChecksum;
};

#endif
