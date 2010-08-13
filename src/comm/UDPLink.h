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

    static const int poll_interval = UDP_POLL_INTERVAL; ///< Polling interval, defined in configuration.h

    bool isConnected() const;
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

    void run();

    int getLinkQuality() const;
    bool isFullDuplex() const;

public slots:
    void setAddress(const QString& address);
    void setPort(quint16 port);
    //    void readPendingDatagrams();

    virtual qint64 read(char* data, qint64 maxLength);
    /**
     * @brief Write a number of bytes to the interface.
     *
     * @param data Pointer to the data byte array
     * @param size The size of the bytes array
     **/
    virtual qint64 write(const char* data, qint64 length);
    bool open();
    bool close();

protected slots:
    void emitBytesReady();
    //void checkForBytes();

protected:
    QHostAddress host;
    quint16 port;
    QUdpSocket* socket;
    bool connectState;
    QList<QHostAddress> hosts;
    QList<quint16> ports;

    quint64 bitsSentTotal;
    quint64 bitsSentCurrent;
    quint64 bitsSentMax;
    quint64 bitsReceivedTotal;
    quint64 bitsReceivedCurrent;
    quint64 bitsReceivedMax;
    quint64 connectionStartTime;
    mutable QMutex statisticsMutex;
    QMutex dataMutex;

signals:
    // Signals are defined by LinkInterface

};

#endif // UDPLINK_H
