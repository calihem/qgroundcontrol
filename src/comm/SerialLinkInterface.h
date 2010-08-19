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

#ifndef SERIALLINKINTERFACE_H_
#define SERIALLINKINTERFACE_H_

#include <LinkInterface.h>
#include <qextserialbase.h>
#include <QString>


class SerialLinkInterface : public LinkInterface {
	Q_OBJECT

	public:
		SerialLinkInterface(int id = -1, ReadingMode readingMode = AutoReading);
		virtual const QString& getPortName() const = 0;
		virtual BaudRateType getBaudRate() const = 0;
		virtual FlowType getFlowControl() const = 0;
		virtual ParityType getParity() const = 0;
		virtual DataBitsType getDataBits() const = 0;
		virtual StopBitsType getStopBits() const = 0;

	public slots:
		virtual void setPortName(const QString& portName) = 0;
		virtual void setBaudRate(BaudRateType baudrateType) = 0;
		virtual void setFlowControl(FlowType flowType) = 0;
		virtual void setParity(ParityType parityType) = 0;
		virtual void setDataBits(DataBitsType dataBitsType) = 0;
		virtual void setStopBits(StopBitsType stopBitsType) = 0;
};

inline SerialLinkInterface::SerialLinkInterface(int id, ReadingMode readingMode) :
	LinkInterface(id, readingMode)
{
}

#endif // SERIALLINKINTERFACE_H_
