/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

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
 *   @brief Definition of the MAVLink protocol
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef MAVLINKPROTOCOL_H_
#define MAVLINKPROTOCOL_H_

#include <QObject>
#include <QMutex>
#include <QString>
#include <QTimer>
#include <QFile>
#include <QPointer>
#include <QMap>
#include <QByteArray>
#include "ProtocolInterface.h"
#include "LinkInterface.h"
#include <mavlink.h>
#include "MG.h"


/**
 * MAVLink micro air vehicle protocol reference implementation.
 *
 **/
class MAVLinkProtocol : public ProtocolInterface {
	Q_OBJECT

	public:
		MAVLinkProtocol();
		~MAVLinkProtocol();

		void run();
		/**
		 * @brief The auto heartbeat emission rate in Hertz
		 * @return heartbeat rate in Hertz
		 */
		int getHeartbeatRate();
		/** @brief Get heartbeat state */
		bool heartbeatsEnabled();
		/** @brief Get logging state */
		bool loggingEnabled();
		/** @brief Get the name of the packet log file */
		static const QString& getLogfileName();

	public slots:
		/**
		 * @brief Receive bytes from a communication interface
		 *
		 * The bytes are copied by calling the LinkInterface::readBytes() method.
		 * This method parses all incoming bytes and constructs a MAVLink packet.
		 * It can handle multiple links in parallel, as each link has it's own buffer/
		 * parsing state machine.
		 * @param linkID The interface to read from
		 * @see LinkInterface
		 **/
		virtual void readFromLink(int linkID);

		/**
		 * @brief Handle bytes from communication interface
		 *
		 * @param linkID The ID of the link the byte comes from
		 * @param data received bytes
		 */
		virtual void handleLinkInput(int linkID, const QByteArray& data);

		/**
		 * @brief Send MAVLink message through connected links 
		 * @param message message to send
		 */
		void sendMessage(const mavlink_message_t& message);

		/**
		 * @brief Set the rate at which heartbeats are emitted
		 *
		 * The default rate is 1 Hertz.
		 * @param rate heartbeat rate in hertz (times per second)
		 */
		void setHeartbeatRate(int rate);

		/**
		 * @brief Enable / disable the heartbeat emission
		 * @param enabled true to enable heartbeats emission at heartbeatRate, false to disable
		 */
		void enableHeartbeats(bool enabled);

		/** @brief Enable/disable binary packet logging */
		void enableLogging(bool enabled);

		/**
		 * @brief Send an extra heartbeat to all connected units
		 *
		 * The heartbeat is sent out of order and does not reset the
		 * periodic heartbeat emission. It will be just sent in addition.
		 * @return mavlink_message_t heartbeat message sent on serial link
		 */
		void sendHeartbeat();

	protected:
		uint8_t sendBuffer[MAVLINK_MAX_PACKET_LEN];
		QMutex sendMutex;
		QTimer heartbeatTimer;     ///< Timer to emit heartbeats
		int heartbeatRate;         ///< Heartbeat rate, controls the timer interval
		bool m_heartbeatsEnabled;  ///< Enabled/disable heartbeat emission
		QPointer<QFile> logFile;   ///< Logfile
		QMutex receiveMutex;       ///< Mutex to protect receiveBytes function
		QMutex parsingMutex;       ///< Mutex to protect parsing of data
		int lastIndex[256][256];
		int totalReceiveCounter;
		int totalLossCounter;
		int currReceiveCounter;
		int currLossCounter;

	signals:
		/** @brief Message received and directly forwarded via signal */
		void messageReceived(const mavlink_message_t& message);
		/** @brief Emitted if heartbeat emission mode is changed */
		void heartbeatChanged(bool heartbeats);
		/** @brief Emitted if logging is started / stopped */
		void loggingChanged(bool enabled);
};

inline bool MAVLinkProtocol::heartbeatsEnabled()
{
	return m_heartbeatsEnabled;
}

inline bool MAVLinkProtocol::loggingEnabled()
{
	return (logFile != NULL);
}

inline int MAVLinkProtocol::getHeartbeatRate()
{
	return heartbeatRate;
}

#endif // MAVLINKPROTOCOL_H_
