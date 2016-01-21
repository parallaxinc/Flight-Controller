#include "packet.h"

packet::packet()
{
    mode = 0;
    len = 0;
    index = 0;
}

quint8 packet::GetByte(void)
{
    return data[index++];
}

qint16 packet::GetShort(void)
{
    union {
        char c[2];
        qint16 i;
    };
    c[0] = data[index+0];
    c[1] = data[index+1];
    index += 2;
    return i;
}

qint32 packet::GetInt(void)
{
    union {
        char c[4];
        int  i;
    };
    c[0] = data[index+0];
    c[1] = data[index+1];
    c[2] = data[index+2];
    c[3] = data[index+3];
    index += 4;
    return i;
}


float packet::GetFloat(void)
{
    union {
        float f;
        int   i;
    };

    i = GetInt();
    return f;
}
