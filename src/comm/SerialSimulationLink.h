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

#ifndef _SERIALSIMULATIONLINK_H_
#define _SERIALSIMULATIONLINK_H_

#include <QString>
#include <QByteArray>
#include <QFile>
#include <QTimer>
#include <QTextStream>
#include <QMutex>
#include <SerialLinkInterface.h>
#include <SerialLink.h>

/**
 * The serial simulation link class simulates a robot connected to the groundstation.
 * It can be either file-based by reading telemetry messages from a file or realtime
 * by communicating with a simulation.
 *
 **/
class SerialSimulationLink : public SerialLinkInterface {
	Q_OBJECT

public:
        SerialSimulationLink(QFile* inputFile=NULL, QFile* outputFile=NULL, int sendrate=20);
	~SerialSimulationLink();

	bool isConnected() const;
	qint64 bytesAvailable() const;

	void run();

	/* Extensive statistics for scientific purposes */
	qint64 getNominalDataRate() const;
	qint64 getTotalUpstream() const;
	qint64 getShortTermUpstream() const;
	qint64 getCurrentUpstream() const;
	qint64 getMaxUpstream() const;
	qint64 getTotalDownstream() const;
	qint64 getShortTermDownstream() const;
	qint64 getCurrentDownstream() const;
	qint64 getMaxDownstream() const;
	qint64 getBitsSent() const;
	qint64 getBitsReceived() const;

	void enableLoopBackMode(SerialLink* loop);
	
		virtual const QString& getPortName() const;
		virtual BaudRateType getBaudRate() const;
		virtual FlowType getFlowControl() const;
		virtual ParityType getParity() const;
		virtual DataBitsType getDataBits() const;
		virtual StopBitsType getStopBits() const;


	int getLinkQuality() const;
	bool isFullDuplex() const;

	public slots:
		virtual void setPortName(const QString& portName);
		virtual void setBaudRate(BaudRateType baudrateType);
		virtual void setFlowControl(FlowType flowType);
		virtual void setParity(ParityType parityType);
		virtual void setDataBits(DataBitsType dataBitsType);
		virtual void setStopBits(StopBitsType stopBitsType);

	void readLine();
        qint64 write(char *bytes, qint64 length);
	qint64 read(char *bytes, qint64 maxLength);

	bool open();
	bool close();

	void setMaximumTimeNoise(int milliseconds);

protected:
	QString portName;
	QTimer timer;
	SerialLink* loopBack;
	/** File which contains the input data (simulated robot messages) **/
	QFile* simulationFile;
	/** File where the commands sent by the groundstation are stored **/
	QFile* receiveFile;
	QTextStream stream;
	QTextStream* fileStream;
	QTextStream* outStream;
	/** Buffer for next line. The next line is read ahead */
	QByteArray lineBuffer;
	/** Buffer which can be read from connected protocols through readBytes(). **/
	QByteArray readyBuffer;
	mutable QMutex readyBufferMutex;
	bool _isConnected;
	quint64 rate;
	int maxTimeNoise;
	quint64 lastSent;

	void addTimeNoise();

signals:
	//void connected();
	//void disconnected();
	//void bytesReady(LinkInterface *link);

};

inline const QString& SerialSimulationLink::getPortName() const {
	return portName;
}

inline BaudRateType SerialSimulationLink::getBaudRate() const
{
	return BAUD115200;
}

inline FlowType SerialSimulationLink::getFlowControl() const
{
	return FLOW_OFF;
}

inline ParityType SerialSimulationLink::getParity() const
{
	return PAR_NONE;
}

inline DataBitsType SerialSimulationLink::getDataBits() const
{
	return DATA_8;
}

inline StopBitsType SerialSimulationLink::getStopBits() const
{
	return STOP_1;
}

inline void SerialSimulationLink::setPortName(const QString& portName)
{
	SerialSimulationLink::portName = portName;
}

inline void SerialSimulationLink::setBaudRate(BaudRateType baudrateType)
{
	Q_UNUSED(baudrateType);
}

inline void SerialSimulationLink::setFlowControl(FlowType flowType)
{
	Q_UNUSED(flowType);
}

inline void SerialSimulationLink::setParity(ParityType parityType)
{
	Q_UNUSED(parityType);
}

inline void SerialSimulationLink::setDataBits(DataBitsType dataBitsType)
{
	Q_UNUSED(dataBitsType);
}

inline void SerialSimulationLink::setStopBits(StopBitsType stopBitsType)
{
	Q_UNUSED(stopBitsType);
}

#endif // _SERIALSIMULATIONLINK_H_
