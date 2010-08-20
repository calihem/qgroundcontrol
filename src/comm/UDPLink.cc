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
 *   @brief Definition of UDP connection (server) for unmanned vehicles
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <QTimer>
#include <QList>
#include <QDebug>
#include <QMutexLocker>
#include <iostream>
#include "UDPLink.h"
#include "MG.h"

UDPLink::UDPLink(QHostAddress host, quint16 port) :
	LinkInterface(-1, AutoReading),
	socket(this),
	hostAddress(host),
	hostPort(port)
{
	// Set name
	name = tr("UDP Link");
}

UDPLink::~UDPLink()
{
	close();
}

void UDPLink::run()
{
}

qint64 UDPLink::write(const char* data, qint64 size)
{
	qint64 writtenBytes = 0;

	// Broadcast to all connected systems
	QList< QPair<QHostAddress, quint16> >::iterator i;
	for (i = broadcastList.begin(); i != broadcastList.end(); ++i)
	{
		writtenBytes = socket.writeDatagram(data, size, i->first, i->second);
// 		qDebug() << "Sent UDP datagramm of size " << writteBytes
// 			<< " to " << i->first << ":" << i->second ;
	}
	
	// Increase write counter (writing is counted only once)
	bitsSentTotal += writtenBytes * 8;

	return writtenBytes;
}

qint64 UDPLink::read(char* data, qint64 maxLength)
{
	QPair< QHostAddress, quint16> sender;
    
	qint64 datagramSize = socket.pendingDatagramSize();
	if (datagramSize > maxLength)
	    qDebug() << __FILE__ << __LINE__ << " UDP datagram overflow, allowed to read less bytes than datagram size";

	qint64 bytesRead = socket.readDatagram(data, maxLength, &sender.first, &sender.second);

	// Construct new QByteArray of size bytesRead without copying data
	QByteArray datagram = QByteArray::fromRawData(data, bytesRead);
	// Emit the new data
	emit dataReceived(getID(), datagram);

	// Increase read counter
	bitsReceivedTotal += bytesRead * 8;

//    // Echo data for debugging purposes
//    std::cerr << __FILE__ << __LINE__ << "Received datagram:" << std::endl;
//    int i;
//    for (i=0; i<s; i++)
//    {
//        unsigned int v=data[i];
//        fprintf(stderr,"%02x ", v);
//    }
//    std::cerr << std::endl;


	// Add sender to broadcast list if not yet present
	if ( !broadcastList.contains(sender) )
	{
		broadcastList.append(sender);
	}

	return bytesRead;
}

void UDPLink::readPendingDatagrams()
{
	QPair< QHostAddress, quint16> sender;
	QByteArray datagram;

	while (socket.hasPendingDatagrams())
	{
		datagram.resize(socket.pendingDatagramSize());
		socket.readDatagram(datagram.data(), datagram.size(), &sender.first, &sender.second);
		emit dataReceived(getID(), datagram);

		// Increase read counter
		bitsReceivedTotal += datagram.size() * 8;
	}

	// Add sender to broadcast list if not yet present
	if ( !broadcastList.contains(sender) )
	{
		broadcastList.append(sender);
	}
}

bool UDPLink::close()
{
	if ( isRunning() )
	{ //terminate thread
		loopForever = false;
		wait();
	}
	if (socket.state() == QAbstractSocket::BoundState)
	{
		socket.flush();
		socket.close();

		emit closed();
		emit opened(false);
	}

	return !isConnected();
}

bool UDPLink::open()
{
	if ( socket.bind(hostAddress, hostPort) )
	{
		emit opened(true);
		emit opened();
		connectionStartTime = MG::TIME::getGroundTimeNow();

		if (readingMode == AutoReading)
		{
			connect(&socket, SIGNAL(readyRead()),
				this, SLOT(readPendingDatagrams()));
		}
		else if (readingMode == ManualReading)
		{
			connect(&socket, SIGNAL(readyRead()),
				this, SLOT(emitReadyRead()));
		}
	}
	else
	{
		qDebug() << "UDP Binding to " << hostAddress << " : " << hostPort << " failed";
	}

	return isConnected();
}

bool UDPLink::isConnected() const
{
    return (socket.state() == QAbstractSocket::BoundState);
}

qint64 UDPLink::getNominalDataRate() const
{
	return 54000000; // 54 Mbit
}

qint64 UDPLink::getTotalUpstream() const {
    statisticsMutex.lock();
    qint64 totalUpstream = bitsSentTotal / ((MG::TIME::getGroundTimeNow() - connectionStartTime) / 1000);
    statisticsMutex.unlock();
    return totalUpstream;
}

qint64 UDPLink::getCurrentUpstream() const {
    return 0; // TODO
}

qint64 UDPLink::getMaxUpstream() const {
    return 0; // TODO
}

qint64 UDPLink::getBitsSent() const {
    return bitsSentTotal;
}

qint64 UDPLink::getBitsReceived() const {
    return bitsReceivedTotal;
}

qint64 UDPLink::getTotalDownstream() const {
    statisticsMutex.lock();
    qint64 totalDownstream = bitsReceivedTotal / ((MG::TIME::getGroundTimeNow() - connectionStartTime) / 1000);
    statisticsMutex.unlock();
    return totalDownstream;
}

qint64 UDPLink::getCurrentDownstream() const {
    return 0; // TODO
}

qint64 UDPLink::getMaxDownstream() const {
    return 0; // TODO
}

bool UDPLink::isFullDuplex() const {
    return true;
}

int UDPLink::getLinkQuality() const {
    /* This feature is not supported with this interface */
    return -1;
}
