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
 *   @brief Brief Description
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef SERIALLINK_H
#define SERIALLINK_H

#include "SerialLinkInterface.h"
#include <QMutex>
#include <qextserialport.h>
#include <configuration.h>

/**
 * @brief The SerialLink class provides cross-platform access to serial links.
 * It takes care of the link management and provides a common API to higher
 * level communication layers. It is implemented as a wrapper class for a thread
 * that handles the serial communication. All methods have therefore to be thread-
 * safe.
 *
 */
class SerialLink : public SerialLinkInterface {
	Q_OBJECT

	public:
		SerialLink(const QString& portname = "", BaudRateType baudrate=BAUD57600, FlowType flow=FLOW_OFF, ParityType parity=PAR_NONE, DataBitsType dataBits=DATA_8, StopBitsType stopBits=STOP_1);
		~SerialLink();

		static const int poll_interval = SERIAL_POLL_INTERVAL; ///< Polling interval, defined in configuration.h
		/**
		 * @brief Check if connection is active.
		 *
		 * @return True if link is connected, false otherwise.
		 **/
		virtual bool isConnected() const;
		/**
		 * @brief Get the number of bytes to read.
		 *
		 * @return The number of bytes to read
		 **/
		qint64 bytesAvailable() const;
		/**
		 * @brief The port handle
		 */
		virtual const QString& getPortName() const;
		virtual BaudRateType getBaudRate() const;
		virtual FlowType getFlowControl() const;
		virtual ParityType getParity() const;
		virtual DataBitsType getDataBits() const;
		virtual StopBitsType getStopBits() const;

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
		virtual void setPortName(const QString& portName);
		virtual void setBaudRate(BaudRateType baudrateType);
		virtual void setFlowControl(FlowType flowType);
		virtual void setParity(ParityType parityType);
		virtual void setDataBits(DataBitsType dataBitsType);
		virtual void setStopBits(StopBitsType stopBitsType);

		/**
		 * @brief Read a number of bytes from the interface.
		 *
		 * @param data Pointer to the data byte array to write the bytes to
		 * @param maxLength The maximum number of bytes to write
		 **/
		qint64 read(char* data, qint64 maxLength);
		/**
		 * @brief Write a number of bytes to the interface.
		 *
		 * @param data Pointer to the data byte array
		 * @param size The size of the bytes array
		 **/
		qint64 write(const char* data, qint64 length);
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

	protected slots:

	protected:
		QString porthandle;
		QextSerialPort port;
		int timeout;

		quint64 bitsSentTotal;
		quint64 bitsSentShortTerm;
		quint64 bitsSentCurrent;
		quint64 bitsSentMax;
		quint64 bitsReceivedTotal;
		quint64 bitsReceivedShortTerm;
		quint64 bitsReceivedCurrent;
		quint64 bitsReceivedMax;
		quint64 connectionStartTime;
		mutable QMutex statisticsMutex;
		QMutex dataMutex;
		/**
		 * @brief This function is called indirectly by the connect() call.
		 *
		 * The connect() function starts the thread and indirectly calls this method.
		 *
		 * @return True if the connection could be established, false otherwise
		 * @see connect() For the right function to establish the connection.
		 **/
		bool hardwareConnect();
		/**
		 * @brief Runs the thread
		 *
		 **/
		void run();


	signals:
		// Signals are defined by LinkInterface

};

// ----------------------------------------------------------------------------
// Inline Implementations
// ----------------------------------------------------------------------------
inline qint64 SerialLink::bytesAvailable() const
{
	return port.bytesAvailable();
}

inline bool SerialLink::isConnected() const
{
	return port.isOpen();
}

inline const QString& SerialLink::getPortName() const
{
	return porthandle;
}

inline BaudRateType SerialLink::getBaudRate() const
{
	return port.baudRate();
}

inline FlowType SerialLink::getFlowControl() const
{
	return port.flowControl();
}

inline ParityType SerialLink::getParity() const
{
	return port.parity();
}

inline DataBitsType SerialLink::getDataBits() const
{
	return port.dataBits();
}

inline StopBitsType SerialLink::getStopBits() const
{
	return port.stopBits();
}

inline void SerialLink::setBaudRate(BaudRateType baudrateType)
{
	port.setBaudRate(baudrateType);
}

inline void SerialLink::setFlowControl(FlowType flowType)
{
	port.setFlowControl(flowType);
}

inline void SerialLink::setParity(ParityType parityType)
{
	port.setParity(parityType);
}

inline void SerialLink::setDataBits(DataBitsType dataBitsType)
{
	port.setDataBits(dataBitsType);
}

inline void SerialLink::setStopBits(StopBitsType stopBitsType)
{
	port.setStopBits(stopBitsType);
}

#endif // SERIALLINK_H
