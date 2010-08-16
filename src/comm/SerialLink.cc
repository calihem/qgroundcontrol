/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Cross-platform support for serial ports
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <QTimer>
#include <QDebug>
#include <QMutexLocker>
#include "SerialLink.h"
#include <MG.h>
#ifdef _WIN32
#include "windows.h"
#endif


SerialLink::SerialLink(const QString& portname, BaudRateType baudrate, FlowType flow, ParityType parity, DataBitsType dataBits, StopBitsType stopBits) :
	SerialLinkInterface(-1, AutoReading),
	porthandle( portname.trimmed() ),
	port(porthandle, QextSerialPort::Polling),
	timeout(50)
{
	// Disable termination for this thread
	setTerminationEnabled(false);

#ifdef _WIN32
	// Port names above 20 need the network path format - if the port name is not already in this format
	// catch this special case
	if (porthandle.size() > 0 && !porthandle.startsWith("\\"))
	{
		// Append \\.\ before the port handle. Additional backslashes are used for escaping.
		porthandle = "\\\\.\\" + porthandle;
		port.setName(porthandle);
	}
#endif
	// Set the link name
	if (porthandle == "")
		name = tr("Serial Link");
	else
		name = porthandle;

	// Configure port
	port.setBaudRate(baudrate);
	port.setFlowControl(flow);
	port.setParity(parity);
	port.setDataBits(dataBits);
	port.setStopBits(stopBits);
	connect(&port, SIGNAL(aboutToClose()), this, SIGNAL(closed()));

}

SerialLink::~SerialLink()
{
	close();
}

void SerialLink::run() {
	qint64 bytesRead;
	qint64 bytesToRead;

	char buffer[1024];

	// Initialize the connection
	hardwareConnect();

	loopForever = true;
	while(loopForever)
	{
		// Do nothing while port is not open
		if (!port.isOpen() )
		{
			emit closed();
			loopForever = false;
			break;
		}
		// Block until new data is available for reading or timeout is reached
		port.waitForReadyRead(timeout);
		
		dataMutex.lock();

		if (readingMode == AutoReading)
		{
			bytesRead = port.read(buffer, 1024);
			if (bytesRead > 0)
			{
				// Construct new QByteArray of size dataRead without copying buffer
				QByteArray data = QByteArray::fromRawData(buffer, bytesRead);
				// Emit the new data
				emit dataReceived(id, data);
				// Increase read counter
				bitsReceivedTotal += bytesRead * 8;
			}
		}
		else if (readingMode == ManualReading)
		{
			bytesToRead = port.bytesAvailable();
			if (bytesToRead > 0)
			{
				emit readyRead(id);
			}
		}
		dataMutex.unlock();
	}
}

qint64 SerialLink::write(const char* data, qint64 size)
{
	qint64 writtenBytes = 0;

	if ( port.isOpen() )
	{
		writtenBytes = port.write(data, size);
// 		qDebug() << "Transmitted " << writtenBytes << "bytes:";

		// Increase write counter
		bitsSentTotal += writtenBytes * 8;
	}

	return writtenBytes;
}

qint64 SerialLink::read(char* data, qint64 maxLength)
{
	qint64 bytesRead;
	qint64 bytesToRead;

	if ( port.isOpen() )
	{
		dataMutex.lock();

		bytesToRead = port.bytesAvailable();
		if(bytesToRead > 0)
		{
			// Read as much data in buffer as possible without overflow
			bytesRead = port.read(data, maxLength);

			// Increase read counter
			bitsReceivedTotal += bytesRead * 8;
		}

		dataMutex.unlock();
	}
    
	return bytesRead;
}

bool SerialLink::close() {
	if( isRunning() )
	{ //terminate thread
		loopForever = false;
		wait();
	}
	
	dataMutex.lock();
	port.flush();
	port.close();
	dataMutex.unlock();

	emit closed();
	emit opened(false);

	return !port.isOpen();
}

bool SerialLink::open()
{
//     qDebug() << "CONNECTING LINK: " << __FILE__ << __LINE__ << "with settings" << porthandle << baudrate << dataBits << parity << stopBits;
	if ( isConnected() )
	{
		close();
	}

	if ( !isRunning() )
	{
		start(LowPriority);
	}
	else
	{
		qDebug() << "FIXME: " << __FILE__ << "[" << __LINE__ << "] Thread is still running";
	}

	return port.isOpen();
}

bool SerialLink::hardwareConnect()
{
	qDebug() << "Opening serial port" << porthandle;


	port.open(QIODevice::ReadWrite);

	statisticsMutex.lock();
	connectionStartTime = MG::TIME::getGroundTimeNow();
	statisticsMutex.unlock();

	bool connectionUp = isConnected();
	if(connectionUp)
	{
		emit opened();
		emit opened(true);
	}

	return connectionUp;
}

qint64 SerialLink::getNominalDataRate() const
{
	switch (port.baudRate())
	{
		case BAUD50:
			return 50;
		case BAUD75:
			return 75;
		case BAUD110:
			return 110;
		case BAUD134:
			return 134;
		case BAUD150:
			return  150;
		case BAUD200:
			return  200;
		case BAUD300:
			return  300;
		case BAUD600:
			return  600;
		case BAUD1200:
			return  1200;
		case BAUD1800:
			return  1800;
		case BAUD2400:
			return  2400;
		case BAUD4800:
			return  4800;
		case BAUD9600:
			return  9600;
		case BAUD14400:
			return  14400;
		case BAUD19200:
			return  19200;
		case BAUD38400:
			return  38400;
		case BAUD56000:
			return  56000;
		case BAUD57600:
			return  57600;
		case BAUD76800:
			return  76800;
		case BAUD115200:
			return  115200;
		case BAUD128000:
			return  128000;
		case BAUD256000:
			return  256000;
	}
	return 0;
}

qint64 SerialLink::getTotalUpstream() const {
    statisticsMutex.lock();
    return bitsSentTotal / ((MG::TIME::getGroundTimeNow() - connectionStartTime) / 1000);
    statisticsMutex.unlock();
}

qint64 SerialLink::getCurrentUpstream() const {
    return 0; // TODO
}

qint64 SerialLink::getMaxUpstream() const {
    return 0; // TODO
}

qint64 SerialLink::getBitsSent() const {
    return bitsSentTotal;
}

qint64 SerialLink::getBitsReceived() const {
    return bitsReceivedTotal;
}

qint64 SerialLink::getTotalDownstream() const {
    statisticsMutex.lock();
    return bitsReceivedTotal / ((MG::TIME::getGroundTimeNow() - connectionStartTime) / 1000);
    statisticsMutex.unlock();
}

qint64 SerialLink::getCurrentDownstream() const {
    return 0; // TODO
}

qint64 SerialLink::getMaxDownstream() const {
    return 0; // TODO
}

bool SerialLink::isFullDuplex() const {
    /* Serial connections are always half duplex */
    return false;
}

int SerialLink::getLinkQuality() const {
    /* This feature is not supported with this interface */
    return -1;
}

void SerialLink::setPortName(const QString& portName)
{
	if (portName.trimmed().length() > 0)
	{
		bool reconnect = false;
		if (isConnected())
		{
			close();
			reconnect = true;
		}
		porthandle = portName.trimmed();
		setName(tr("Serial Port ") + portName.trimmed());
#ifdef _WIN32
		// Port names above 20 need the network path format - if the port name is not already in this format
		// catch this special case
		if (!porthandle.startsWith("\\"))
		{
			// Append \\.\ before the port handle. Additional backslashes are used for escaping.
			porthandle = "\\\\.\\" + this->porthandle;
		}
#endif
		port.setPortName(porthandle);
		if(reconnect) open();
	}
}
