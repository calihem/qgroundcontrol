/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

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
 *   @brief Implementation of the MAVLink protocol
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <inttypes.h>
#include <iostream>

#include <QDebug>
#include <QTime>
#include <QApplication>

#include "MAVLinkProtocol.h"
#include "UASFactory.h"
// #include "UASInterface.h"
#include "UASManager.h"
// #include "UASInterface.h"
#include "configuration.h"
#include "ProtocolStack.h"
#include <mavlink.h>
#include "QGC.h"

/**
 * The default constructor will create a new MAVLink object sending heartbeats at
 * the MAVLINK_HEARTBEAT_DEFAULT_RATE to all connected links.
 */
MAVLinkProtocol::MAVLinkProtocol() :
	heartbeatTimer(this),
	heartbeatRate(MAVLINK_HEARTBEAT_DEFAULT_RATE),
	m_heartbeatsEnabled(false),
	totalReceiveCounter(0),
	totalLossCounter(0),
	currReceiveCounter(0),
	currLossCounter(0)

{
	name = tr("MAVLink Protocol");

	start(QThread::LowPriority);

	// Start heartbeat timer, emitting a heartbeat at the configured rate
	connect(&heartbeatTimer, SIGNAL(timeout()), this, SLOT(sendHeartbeat()));
	heartbeatTimer.start(1000/heartbeatRate);

	memset(lastIndex, -1, sizeof(int)*256*256);
}

MAVLinkProtocol::~MAVLinkProtocol()
{
	if (logFile)
	{
		logFile->close();
		delete logFile;
	}
}

void MAVLinkProtocol::run()
{
	//TODO: put parsing of bytestream and forwarding to UAS in thread mainloop
}

const QString& MAVLinkProtocol::getLogfileName()
{
	static QString logfilename = QCoreApplication::applicationDirPath()+"/mavlink.log";
	return logfilename;
}

void MAVLinkProtocol::readFromLink(int linkID)
{
	receiveMutex.lock();

	LinkInterface *link = ProtocolStack::instance().getLink(linkID);
	if (!link) return;
	
	// Prepare buffer
	static const int maxlen = 4096 * 100;
	static char buffer[maxlen];
	qint64 bytesToRead = link->bytesAvailable();

	if (bytesToRead > maxlen) bytesToRead = maxlen;
	// Get all data at once, let link read the bytes in the buffer array
	qint64 bytesRead = link->read(buffer, bytesToRead);

	// Construct new QByteArray of size bytesRead without copying buffer
	QByteArray data = QByteArray::fromRawData(buffer, bytesRead);
	
	handleLinkInput(linkID, data);

	receiveMutex.unlock();
}

void MAVLinkProtocol::handleLinkInput(int linkID, const QByteArray& data)
{
	if (linkID < 0 || linkID >= MAVLINK_COMM_NB_HIGH)
	{
		qDebug() << "Link ID out of range";
		return;
	}

	parsingMutex.lock();

	mavlink_message_t message;
	mavlink_status_t status;

	for (int position = 0; position < data.size(); position++)
	{
		unsigned int decodeState = mavlink_parse_char(linkID, (uint8_t)data.at(position), &message, &status);

		if (decodeState == 1)
		{
			// Log data
			if (logFile)
			{
				uint8_t buf[MAVLINK_MAX_PACKET_LEN];
				quint64 time = MG::TIME::getGroundTimeNowUsecs();
				memcpy(buf, (void*)&time, sizeof(quint64));
				mavlink_msg_to_send_buffer(buf+sizeof(quint64), &message);
				logFile->write((const char*) buf);
				qDebug() << "WROTE LOGFILE";
			}

			// ORDER MATTERS HERE!
			// If the matching UAS object does not yet exist, it has to be created
			// before emitting the packetReceived signal
			UASInterface* uas = UASManager::instance()->getUASForId(message.sysid);

			// Check and (if necessary) create UAS object
			if (uas == NULL && message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
			{
				// ORDER MATTERS HERE!
				// The UAS object has first to be created and connected,
				// only then the rest of the application can be made aware
				// of its existence, as it only then can send and receive
				// it's first messages.

				// FIXME Current debugging
				// check if the UAS has the same id like this system
				if (message.sysid == MG::SYSTEM::ID)
				{
					qDebug() << "WARNING\nWARNING\nWARNING\nWARNING\nWARNING\nWARNING\nWARNING\n\n RECEIVED MESSAGE FROM THIS SYSTEM WITH ID" << message.msgid << "FROM COMPONENT" << message.compid;
				}

				// Create a new UAS based on the heartbeat received
				// Todo dynamically load plugin at run-time for MAV
				// WIKISEARCH:AUTOPILOT_TYPE_INSTANTIATION

				// First create new UAS object
				// Decode heartbeat message
				mavlink_heartbeat_t heartbeat;
				mavlink_msg_heartbeat_decode(&message, &heartbeat);
				uas = UASFactory::buildUAV( (MAV_AUTOPILOT_TYPE)heartbeat.autopilot, message.sysid);
				if (!uas)
				{
					qDebug() << "Builing of UAS failed";
				}
				else
				{ // builing of uas succesufull
					//FIXME: connection should be the task of UASManager/ ProtocolStackis
					connect(this, SIGNAL(messageReceived(const mavlink_message_t&)),
							uas, SLOT(handleMessage(const mavlink_message_t&)));
					connect(uas, SIGNAL(messageToSend(const mavlink_message_t&)),
						this, SLOT(sendMessage(const mavlink_message_t)) );

					// Now add UAS to "official" list, which makes the whole application aware of it
					UASManager::instance()->addUAS(uas);
				}
			}

			// Only count message if UAS exists for this message
			if (uas != NULL)
			{
				// Increase receive counter
				totalReceiveCounter++;
				currReceiveCounter++;
				qint64 lastLoss = totalLossCounter;
				// Update last packet index
				if (lastIndex[message.sysid][message.compid] == -1)
				{
					lastIndex[message.sysid][message.compid] = message.seq;
				}
				else
				{
					if (lastIndex[message.sysid][message.compid] == 255)
					{
						lastIndex[message.sysid][message.compid] = 0;
					}
					else
					{
						lastIndex[message.sysid][message.compid]++;
					}

					int safeguard = 0;
					//qDebug() << "SYSID" << message.sysid << "COMPID" << message.compid << "MSGID" << message.msgid << "LASTINDEX" << lastIndex[message.sysid][message.compid] << "SEQ" << message.seq;
					while(lastIndex[message.sysid][message.compid] != message.seq && safeguard < 255)
					{
						if (lastIndex[message.sysid][message.compid] == 255)
						{
							lastIndex[message.sysid][message.compid] = 0;
						}
						else
						{
							lastIndex[message.sysid][message.compid]++;
						}
						totalLossCounter++;
						currLossCounter++;
						safeguard++;
					}
				}
				//            if (lastIndex.contains(message.sysid))
				//            {
				//                QMap<int, int>* lastCompIndex = lastIndex.value(message.sysid);
				//                if (lastCompIndex->contains(message.compid))
				//                while (lastCompIndex->value(message.compid, 0)+1 )
				//            }
				//if ()

				// If a new loss was detected or we just hit one 128th packet step
				if (lastLoss != totalLossCounter || (totalReceiveCounter & 0x7F) == 0)
				{
					// Calculate new loss ratio
					// Receive loss
					float receiveLoss = (double)currLossCounter/(double)(currReceiveCounter+currLossCounter);
					receiveLoss *= 100.0f;
					// qDebug() << "LOSSCHANGED" << receiveLoss;
					currLossCounter = 0;
					currReceiveCounter = 0;
					emit receiveLossChanged(message.sysid, receiveLoss);
				}

				// The packet is emitted as a whole, as it is only 255 - 261 bytes short
				// kind of inefficient, but no issue for a groundstation pc.
				// It buys as reentrancy for the whole code over all threads
				emit messageReceived(message);
			}
		}
	}
	parsingMutex.unlock();
}


void MAVLinkProtocol::sendMessage(const mavlink_message_t& message)
{
	sendMutex.lock();

	// Write message into buffer, prepending start sign
	int bufferLen = mavlink_msg_to_send_buffer(sendBuffer, &message);
	// Construct new QByteArray of size dataRead without copying buffer
	QByteArray data = QByteArray::fromRawData((char*)sendBuffer, bufferLen);
	emit dataToSend(data);
	
	sendMutex.unlock();
}

void MAVLinkProtocol::sendHeartbeat()
{
    if (m_heartbeatsEnabled)
    {
        mavlink_message_t beat;
        mavlink_msg_heartbeat_pack(MG::SYSTEM::ID, MG::SYSTEM::COMPID,&beat, OCU, MAV_AUTOPILOT_GENERIC);
        sendMessage(beat);
    }
}

void MAVLinkProtocol::enableHeartbeats(bool enabled)
{
    m_heartbeatsEnabled = enabled;
    emit heartbeatChanged(enabled);
}

void MAVLinkProtocol::enableLogging(bool enabled)
{
	if (enabled && !logFile)
	{
		logFile = new QFile( getLogfileName() );
		logFile->open(QIODevice::WriteOnly | QIODevice::Append);
	}
	else
	{
		logFile->close();
		delete logFile;
	}
}


void MAVLinkProtocol::setHeartbeatRate(int rate)
{
    heartbeatRate = rate;
    heartbeatTimer.setInterval(1000/heartbeatRate);
}

