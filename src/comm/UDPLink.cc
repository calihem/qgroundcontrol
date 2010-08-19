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
	host(host),
	port(port),
	socket(NULL),
	connectState(false)
{
    // Set name
    name = tr("UDP Link");
}

UDPLink::~UDPLink()
{
    close();
}

/**
 * @brief Runs the thread
 *
 **/
void UDPLink::run()
{
}

void UDPLink::setAddress(const QString& address)
{
    Q_UNUSED(address);
    // FIXME TODO Implement address
    //socket->setLocalAddress(QHostAddress(address));
}

void UDPLink::setPort(quint16 port)
{
    UDPLink::port = port;
}


qint64 UDPLink::write(const char* data, qint64 size)
{
    // Broadcast to all connected systems
    //QList<QHostAddress>::iterator h;
    // for (h = hosts.begin(); h != hosts.end(); ++h)

    qint64 writtenBytes = 0;
    for (int h = 0; h < hosts.size(); h++)
    {
        QHostAddress currentHost = hosts.at(h);
        quint16 currentPort = ports.at(h);
        //        QList<quint16> currentPorts = ports.values(currentHost);
        //        for (int p = 0; p < currentPorts.size(); p++)
        //        {
        //            quint16 currentPort = currentPorts.at(p);
        //qDebug() << "Sent message to " << currentHost << ":" << currentPort << "at" << __FILE__ << ":" << __LINE__;
        writtenBytes = socket->writeDatagram(data, size, currentHost, currentPort);
        //        }
    }


    //if(socket->write(data, size) > 0) {

//    qDebug() << "Transmitted " << size << "bytes:";
//
//    /* Increase write counter */
//    bitsSentTotal += size * 8;
//
//    int i;
//    for (i=0; i<size; i++){
//        unsigned int v=data[i];
//
//        fprintf(stderr,"%02x ", v);
//    }
    //}
    return writtenBytes;
}

/**
 * @brief Read a number of bytes from the interface.
 *
 * @param data Pointer to the data byte array to write the bytes to
 * @param maxLength The maximum number of bytes to write
 **/
qint64 UDPLink::read(char* data, qint64 maxLength)
{
    QHostAddress sender;
    quint16 senderPort;
    
    qint64 datagramSize = socket->pendingDatagramSize();
    if (datagramSize > maxLength)
	    std::cerr << __FILE__ << __LINE__ << " UDP datagram overflow, allowed to read less bytes than datagram size" << std::endl;

    qint64 receivedBytes = socket->readDatagram(data, maxLength, &sender, &senderPort);

    // FIXME TODO Check if this method is better than retrieving the data by individual processes
    QByteArray b(data, receivedBytes);
//     emit bytesReceived(this, b);
	emit dataReceived(id, b);
//    // Echo data for debugging purposes
//    std::cerr << __FILE__ << __LINE__ << "Received datagram:" << std::endl;
//    int i;
//    for (i=0; i<s; i++)
//    {
//        unsigned int v=data[i];
//        fprintf(stderr,"%02x ", v);
//    }
//    std::cerr << std::endl;


    // Add host to broadcast list if not yet present
    if (!hosts.contains(sender))
    {
        hosts.append(sender);
        ports.append(senderPort);
        //        ports.insert(sender, senderPort);
    }
    else
    {
        int index = hosts.indexOf(sender);
        ports.replace(index, senderPort);
    }

    return receivedBytes;
}


/**
 * @brief Get the number of bytes to read.
 *
 * @return The number of bytes to read
 **/
qint64 UDPLink::bytesAvailable() const {
    return socket->pendingDatagramSize();
}

/**
 * @brief Convenience method to emit the bytesReady signal
 **/
void UDPLink::emitBytesReady()
{
//     emit bytesReady(this);
	emit readyRead(id);
}

/**
 * @brief Disconnect the connection.
 *
 * @return True if connection has been disconnected, false if connection couldn't be disconnected.
 **/
bool UDPLink::close()
{
    //FIXME: only emit closed, if state is QAbstractSocket::ConnectedState || QAbstractSocket::BoundState
    delete socket;
    socket = NULL;

    connectState = false;

    emit closed();
    emit opened(false);
    return !connectState;
}

/**
 * @brief Connect the connection.
 *
 * @return True if connection has been established, false if connection couldn't be established.
 **/
bool UDPLink::open()
{
    socket = new QUdpSocket(this);

    //QObject::connect(socket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));
    QObject::connect(socket, SIGNAL(readyRead()), this, SLOT(emitBytesReady()));

    connectState = socket->bind(host, port);

    emit opened(connectState);
    if (connectState)
    {
        emit opened();
        connectionStartTime = MG::TIME::getGroundTimeNow();
    }

    start(HighPriority);
    return connectState;
}

/**
 * @brief Check if connection is active.
 *
 * @return True if link is connected, false otherwise.
 **/
bool UDPLink::isConnected() const {
    return connectState;
}

qint64 UDPLink::getNominalDataRate() const {
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
