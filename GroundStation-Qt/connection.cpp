
#include <QtSerialPort/QSerialPortInfo>
#include <QtDebug>
#include "connection.h"

Connection::Connection(QObject *parent) : QThread(parent)
{
    quit = false;
    connected = false;
    active = true;

    commStat = CS_Initializing;

    sigByteIndex = 0;
    packetByteIndex = 0;

    currentPacket = 0;
    head = tail = 0;
    memset( packetsArray, 0, packetCount * sizeof(void*) );
}

Connection::~Connection()
{
}

void Connection::StartConnection(void)
{
    quit = false;
    connected = false;

	start( QThread::HighPriority );
}

void Connection::StopConnection(void)
{
    mutex.lock();
    quit = true;
    mutex.unlock();
    wait();
}

void Connection::run()
{
    QSerialPort port;
	serial = &port;

    while( !quit )
    {
		Update();
        if( connected == false ) {
            QThread::msleep(20);
        }
    }

    serial = 0;
}

static void dbgOnErr( QSerialPort * serial , const char * context )
{
(void)serial;	// "Use" these params to prevent the compiler from warning
(void)context;

#if 0	// enable for debugging
	QSerialPort::SerialPortError err = serial->error();
	switch( err )
	{
	case QSerialPort::DeviceNotFoundError:
		qDebug() << "DeviceNotFoundError : " << context;
		break;

	case QSerialPort::OpenError:
		qDebug() << "OpenError : " << context;
		break;

	case QSerialPort::WriteError:
		qDebug() << "WriteError : " << context;
		break;

	case QSerialPort::ReadError:
		qDebug() << "ReadError : " << context;
		break;

	case QSerialPort::UnknownError:
		qDebug() << "UnknownError : " << context;
		break;

	case QSerialPort::PermissionError:
		qDebug() << "PermissionError : " << context;
		break;

	case QSerialPort::UnsupportedOperationError:
		qDebug() << "UnsupportedOperation : " << context;
		break;

	case QSerialPort::NoError:
	case QSerialPort::TimeoutError:
		break;

	default:
		break;
	}
	serial->clearError();
#endif
}



void Connection::Update(void)
{
    if( active == false ) return;

    if( !connected ) {

		AttemptConnect();
        if( !connected ) return;
	}

	if( toSend.length() > 0 ) {
		mutex.lock();
		serial->write( toSend );
		toSend.clear();
		mutex.unlock();
		//serial->waitForBytesWritten(1);
	}

	while( serial->waitForReadyRead(1) )
	{
		QByteArray bytes = serial->readAll();

		for( int i=0; i<bytes.length(); i++ ) {
			ProcessByte( (quint8)bytes[i] );

			// Do this to grab new data if it arrives while processing the current set
			// On modern machines we should easily outpace the incoming data, but this
			// helps keep the FTDI buffer from hitting its 64-byte overfill mark on PCs

			if( serial->waitForReadyRead(0) ) {
				bytes += serial->readAll();
			}
		}
	}

	QSerialPort::SerialPortError err = serial->error();
	if( err != QSerialPort::NoError && err != QSerialPort::TimeoutError )
	{
		serial->close();
		connected = false;
		commStat = CS_NoDevice;
	}
}


void Connection::ProcessByte( quint8 b )
{
    if(sigByteIndex < 2)
    {
        if(sigByteIndex == 0)
        {
			if(b == 0x55) {
                sigByteIndex++;
                packetByteIndex = 0;
            }
            return;
        }


        if(sigByteIndex == 1)
        {
			if(b == 0xAA) {
                sigByteIndex++;
                packetByteIndex = 0;
            }
			else {
                sigByteIndex = 0;
            }
			return;
        }
    }


    switch(packetByteIndex)
    {
        case 0:
            if(b > 0x20)
            {	// No such mode - bad data
                sigByteIndex = 0;
                packetByteIndex = 0;
				return;
            }
            currentPacket = new packet();
            currentPacket->mode = b;
            packetByteIndex++;
            return;

        case 1:
            if(b != 0)
            {	// No such mode - bad data
                sigByteIndex = 0;
                packetByteIndex = 0;
                delete currentPacket;
                currentPacket = 0;
				return;
            }
            packetByteIndex++;
            currentChecksum = Checksum( 0, (quint16)0xaa55 );
            currentChecksum = Checksum( currentChecksum, (quint16)currentPacket->mode );
            return;

        case 2:
            currentPacket->len = (short)b;
            packetByteIndex++;
            return;

        case 3:
            currentPacket->len |= (short)(b << 8);
            currentChecksum = Checksum( currentChecksum, (quint16)currentPacket->len );

            if(currentPacket->len > 1024)
            {
                // unreasonable packet size (> 1kb)
                sigByteIndex = 0;
                packetByteIndex = 0;
                delete currentPacket;
                currentPacket = 0;
				return;
            }

            currentPacket->len -= 6;		// Subtract off signature and header size
            if(currentPacket->len < 1)
            {	// Can't have a zero length packet - bad data
                sigByteIndex = 0;
                packetByteIndex = 0;
                delete currentPacket;
                currentPacket = 0;
				return;
            }
            currentPacket->data.reserve(currentPacket->len);
            packetByteIndex++;
            return;

        default:
            currentPacket->data.append(b);
            break;
    }

    packetByteIndex++;
    if(packetByteIndex == (currentPacket->len + 4))	// length + header bytes
    {
        sigByteIndex = 0;
        packetByteIndex = 0;

        // Validate the checksum
        // only keep the packet if they match
        int len = currentPacket->data.length() - 2;

        quint16 check = Checksum( currentChecksum, (quint8*)currentPacket->data.data(), len );
        quint16 sourceCheck = (quint16)((quint8)currentPacket->data[len] | (currentPacket->data[len + 1] << 8));

        if(check == sourceCheck)
        {
            mutex.lock();
            if( packetsArray[head] != 0 ) {
                delete packetsArray[head];
                packetsArray[head] = 0;
            }
            packetsArray[head] = currentPacket;

            head = (head + 1) % packetCount;

            if(tail == head) {
                tail = (tail + 1) % packetCount;		// Throw away oldest data if we fill the buffer
            }
            mutex.unlock();
		}
        else
        {
			delete currentPacket;
            currentPacket = 0;
        }
    }
}


quint16 Connection::Checksum( quint16 checksum, const quint8 * buf, int Length )
{
    for(int i = 0; i < Length; i += 2)
    {
        quint16 val = (quint16)((buf[i] << 0) | (buf[i + 1] << 8));
        checksum = (quint16)(((checksum << 5) | (checksum >> (16 - 5))) ^ val);
    }
    return checksum;
}

quint16 Connection::Checksum( quint16 checksum, quint16 val )
{
    checksum = (quint16)(((checksum << 5) | (checksum >> (16 - 5))) ^ val);
    return checksum;
}


packet * Connection::GetPacket()
{
    QMutexLocker lock(&mutex);

    if(head == tail) return 0;

    packet * p = packetsArray[tail];
    packetsArray[tail] = 0;

    tail = (tail + 1) % packetCount;
    return p;
}


void Connection::Send( quint8 * bytes , int count )
{
	if(connected == false) return;

	QMutexLocker lock(&mutex);
    toSend.append( (char*)bytes, count);
}

void Connection::Reset(void)
{
	if( connected == false ) return;
	QMutexLocker lock(&mutex);
	serial->setDataTerminalReady(true);
	QThread::usleep(100);
	serial->setDataTerminalReady(false);
}

void Connection::AttemptConnect(void)
{
	CommStatus tempStatus = CS_NoDevice;
    connected = false;

    char txBuf[4];

    txBuf[0] = 'E';
    txBuf[1] = 'l';
    txBuf[2] = 'v';
    txBuf[3] = '8';


    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        if( info.portName() == QString("COM3") ) continue;
        if( info.portName() == QString("COM4") ) continue;

		if( quit ) return;

		serial->close();
        serial->setPortName(info.portName());

		// On Windows systems, FTDI USB devices may end up buffering up to 4Kb of
		// data at a time before passing it to the host, if the data comes in faster
		// than 62 bytes per 16ms.  Setting a small buffer and reading often seems to help.
		// (see http://www.qtcentre.org/threads/57633-QSerialPort-read-delay)

		serial->setReadBufferSize(60);
		serial->setSettingsRestoredOnClose(false);

		if( !serial->open(QIODevice::ReadWrite) || !serial->isOpen() ) {
            continue;
        }

#ifdef WIN32	// Only do this on windows systems
		serial->setDataTerminalReady(false);
#endif

		serial->setBaudRate( QSerialPort::Baud115200 );
		serial->setFlowControl( QSerialPort::NoFlowControl );
		serial->setParity( QSerialPort::NoParity );
		serial->setStopBits( QSerialPort::OneStop );

		tempStatus = CS_NoElev8;

        int TestVal = 0;

		for( int i=0; i<10 && !quit; i++ )
        {
            // Send the ELV8 signature
            serial->write(txBuf, 4);
			serial->waitForBytesWritten(5);

			QThread::usleep(20);
            if( serial->waitForReadyRead(50) )
            {
                int bytesAvail = (int)serial->bytesAvailable();
                while( bytesAvail > 0 )
                {
                    char c;
                    if( serial->getChar( &c ) )
                    {
                        TestVal = (TestVal << 8) | (quint8)c;
                        if(TestVal == (int)(('E' << 0) | ('l' << 8) | ('v' << 16) | ('8' << 24)))
                        {
                            //FoundElev8 = true;
                            commStat = CS_Connected;
                            connected = true;
                            emit connectionMade();
                            return;
                        }
                    }
                    bytesAvail--;
                }
            }
        }
    }
	commStat = tempStatus;
}

void Connection::Disconnect(void)
{
    if( serial != 0 && serial->isOpen() ) {
		serial->close();
        connected = false;
    }
}
