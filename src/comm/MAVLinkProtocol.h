/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009, 2010 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

This file is part of the QGROUNDCONTROL project

    QGROUNDCONTROL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QGROUNDCONTROL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Definition of class MAVLinkProtocol
 *   @author Lorenz Meier <mail@qgroundcontrol.org>
 */

#ifndef MAVLINKPROTOCOL_H_
#define MAVLINKPROTOCOL_H_

#include <QObject>
#include <QMutex>
#include <QString>
#include <QTimer>
#include <QFile>
#include <QMap>
#include <QByteArray>
#include "ProtocolInterface.h"
#include "LinkInterface.h"
#include "protocol.h"
#include "mavlink.h"

/**
 * @brief MAVLink micro air vehicle protocol reference implementation.
 *
 * MAVLink is a generic communication protocol for micro air vehicles.
 * for more information, please see the official website.
 * @ref http://pixhawk.ethz.ch/software/mavlink/
 **/
class MAVLinkProtocol : public ProtocolInterface {
    Q_OBJECT

public:
    MAVLinkProtocol();
    ~MAVLinkProtocol();

    void run();
    /** @brief Get the human-friendly name of this protocol */
    QString getName();
    /** @brief Get the system id of this application */
    int getSystemId();
    /** @brief Get the component id of this application */
    int getComponentId();
    /** @brief The auto heartbeat emission rate in Hertz */
    int getHeartbeatRate();
    /** @brief Get heartbeat state */
    bool heartbeatsEnabled(void);
    /** @brief Get logging state */
    bool loggingEnabled(void);
    /** @brief Get the name of the packet log file */
    static QString getLogfileName();

public slots:
    /** @brief Receive bytes from a communication interface */
    void receiveBytes(LinkInterface* link, QByteArray b);
    /** @brief Send MAVLink message through serial interface */
    void sendMessage(mavlink_message_t message);
    /** @brief Send MAVLink message through serial interface */
    void sendMessage(LinkInterface* link, mavlink_message_t message);
    /** @brief Set the rate at which heartbeats are emitted */
    void setHeartbeatRate(int rate);

    /** @brief Enable / disable the heartbeat emission */
    void enableHeartbeats(bool enabled);

    /** @brief Enable/disable binary packet logging */
    void enableLogging(bool enabled);

    /** @brief Send an extra heartbeat to all connected units */
    void sendHeartbeat();

protected:
    QTimer* heartbeatTimer;    ///< Timer to emit heartbeats
    int heartbeatRate;         ///< Heartbeat rate, controls the timer interval
    bool m_heartbeatsEnabled;  ///< Enabled/disable heartbeat emission
    bool m_loggingEnabled;     ///< Enable/disable packet logging
    QFile* m_logfile;           ///< Logfile
    QMutex receiveMutex;       ///< Mutex to protect receiveBytes function
    int lastIndex[256][256];
    int totalReceiveCounter;
    int totalLossCounter;
    int currReceiveCounter;
    int currLossCounter;

signals:
    /** @brief Message received and directly copied via signal */
    void messageReceived(LinkInterface* link, mavlink_message_t message);
    /** @brief Emitted if heartbeat emission mode is changed */
    void heartbeatChanged(bool heartbeats);
    /** @brief Emitted if logging is started / stopped */
    void loggingChanged(bool enabled);
};

#endif // MAVLINKPROTOCOL_H_
