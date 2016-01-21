#ifndef PACKET_H
#define PACKET_H

#include <QByteArray>


class packet
{
public:
    packet();

    quint8 mode;
    quint16 len;
    QByteArray data;
    quint16 index;


    quint8 GetByte(void);
    qint16 GetShort(void);
    qint32 GetInt(void);
    float  GetFloat(void);
};

#endif // PACKET_H


/*
    float GetFloat()
*/
