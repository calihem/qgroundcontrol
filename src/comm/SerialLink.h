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

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QString>
#include <qextserialport.h>
#include <configuration.h>
#include "SerialLinkInterface.h"
#ifdef _WIN32
#include "windows.h"
#endif


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
    //Q_INTERFACES(SerialLinkInterface:LinkInterface)

public:
    SerialLink(QString portname=NULL, BaudRateType baudrate=BAUD57600, FlowType flow=FLOW_OFF, ParityType parity=PAR_NONE, DataBitsType dataBits=DATA_8, StopBitsType stopBits=STOP_1);
    ~SerialLink();

    static const int poll_interval = SERIAL_POLL_INTERVAL; ///< Polling interval, defined in configuration.h

    bool isConnected() const;
    qint64 bytesAvailable() const;

    /**
     * @brief The port handle
     */
    const QString& getPortName() const;
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
    bool setPortName(const QString& portName);
    bool setBaudRate(int rate);
    bool setBaudRateType(int rateIndex);
    bool setFlowType(int flow);
    bool setParityType(int parity);
    bool setDataBitsType(int dataBits);
    bool setStopBitsType(int stopBits);

    qint64 read(char* data, qint64 maxLength);
    /**
     * @brief Write a number of bytes to the interface.
     *
     * @param data Pointer to the data byte array
     * @param size The size of the bytes array
     **/
    qint64 write(const char* data, qint64 length);
    bool open();
    bool close();

protected slots:
    void emitBytesReady();
    void checkForBytes();

protected:
    QextSerialPort* port;
#ifdef _WIN32
    HANDLE winPort;
    DCB winPortSettings;
#endif
    QString porthandle;
    BaudRateType baudrate;
    FlowType flow;
    ParityType parity;
    DataBitsType dataBits;
    StopBitsType stopBits;
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

    bool hardwareConnect();

signals:
    // Signals are defined by LinkInterface

};

#endif // SERIALLINK_H
