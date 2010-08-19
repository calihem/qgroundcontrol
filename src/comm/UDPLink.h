/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
 *   @brief UDP connection (server) for unmanned vehicles
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef UDPLINK_H
#define UDPLINK_H

#include <QString>
#include <QList>
#include <QMap>
#include <QMutex>
#include <QUdpSocket>
#include <LinkInterface.h>
#include <configuration.h>

class UDPLink : public LinkInterface
{
	Q_OBJECT
    //Q_INTERFACES(UDPLinkInterface:LinkInterface)

	public:
		UDPLink(QHostAddress host = QHostAddress::Any, quint16 port = 14550);
		~UDPLink();

		/**
		 * @brief Check if connection is active.
		 *
		 * @return True if link is connected, false otherwise.
		 **/
		bool isConnected() const;
		/**
		 * @brief Get the number of bytes to read.
		 *
		 * @return The number of bytes to read
		 **/
		qint64 bytesAvailable() const;

		int getBaudRate() const;
		int getBaudRateType() const;
		int getFlowType() const;
		int getParityType() const;
		int getDataBitsType() const;
		int getStopBitsType() const;

		/* Extensive statistics for scientific purposes */
		qint64 getNominalDataRate() const;
		qint64 getTotalUpstream() const;
		qint64 getCurrentUpstream() const;
		qint64 getMaxUpstream() const;
		qint64 getTotalDownstream() const;
		qint64 getCurrentDownstream() const;
		qint64 getMaxDownstream() const;
		qint64 getBitsSent() const;
		qint64 getBitsReceived() const;

		int getLinkQuality() const;
		bool isFullDuplex() const;

	public slots:
		void setAddress(const QString& address);
		void setPort(quint16 port);

		/**
		 * @brief Read a number of bytes from the interface.
		 *
		 * @param data Pointer to the data byte array to write the bytes to
		 * @param maxLength The maximum number of bytes to write
		 **/
		virtual qint64 read(char* data, qint64 maxLength);

		/**
		 * @brief Write a number of bytes to the interface.
		 *
		 * @param data Pointer to the data byte array
		 * @param size The size of the bytes array
		 **/
		virtual qint64 write(const char* data, qint64 length);
		
		/**
		 * @brief Connect the connection.
		 *
		 * @return True if connection has been established, false if connection couldn't be established.
		 **/
		bool open();
		
		/**
		 * @brief Disconnect the connection.
		 *
		 * @return True if connection has been disconnected, false if connection couldn't be disconnected.
		 **/
		bool close();

	protected:
		/// The UDP socket
		QUdpSocket socket;
		/// Host address from which datagrams are accepted (default: any)
		QHostAddress hostAddress;
		/// Local port on which UDP socket is listening
		quint16 hostPort;
		/// A list of all senders (address/port pairs) from which datagramms were reiceived 
		QList< QPair<QHostAddress, quint16> > broadcastList;

		// FIXME: move variables to LinkInterface
		quint64 bitsSentTotal;
		quint64 bitsSentCurrent;
		quint64 bitsSentMax;
		quint64 bitsReceivedTotal;
		quint64 bitsReceivedCurrent;
		quint64 bitsReceivedMax;
		quint64 connectionStartTime;
		mutable QMutex statisticsMutex;

		QMutex dataMutex;
		/**
		 * @brief Runs the thread
		 *
		 **/
		void run();

	protected slots:
		void reconnect();
		/**
		 * @brief Convenience method to emit the bytesReady signal
		 **/
		void emitReadyRead();
		void readPendingDatagrams();


	signals:
		// Signals are defined by LinkInterface
};

inline qint64 UDPLink::bytesAvailable() const
{
	return socket.pendingDatagramSize();
}

inline void UDPLink::setAddress(const QString& address)
{
	hostAddress.setAddress(address);
	reconnect();
}

inline void UDPLink::setPort(quint16 port)
{
	hostPort = port;
	reconnect();
}

inline void UDPLink::reconnect()
{
	if (socket.state() ==  QAbstractSocket::BoundState)
	{ //reconnect
		close();
		open();
	}
}

inline void UDPLink::emitReadyRead()
{
	emit readyRead(id);
}

#endif // UDPLINK_H
