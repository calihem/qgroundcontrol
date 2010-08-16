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
 *   @brief Interface class for protocols
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef _PROTOCOLINTERFACE_H_
#define _PROTOCOLINTERFACE_H_

#include <QThread>
#include <QString>
#include "LinkInterface.h"

/**
 * @brief Interface for all protocols.
 *
 * This class defines the interface for
 * communication packets transported by the ProtocolStack.
 * 
 * @see ProtocolStack.
 * 
 **/
class ProtocolInterface : public QThread
{
	Q_OBJECT
	public:
		virtual ~ProtocolInterface() {};
		/** @brief Get the human-friendly name of this protocol */
		virtual const QString& getName() const;

	public slots:
// 		virtual void receiveBytes(LinkInterface* link) = 0;
// 		virtual void readFromLink(int linkID) = 0;
		virtual void handleLinkInput(int linkID, const QByteArray& data) = 0;

	signals:
		/** @brief Update the packet loss from one system */
		void receiveLossChanged(int uasId, float loss);

	protected:
		QString name;
};

// ----------------------------------------------------------------------------
// Inline Implementations
// ----------------------------------------------------------------------------
inline const QString& ProtocolInterface::getName() const
{
	return name;
}
#endif // _PROTOCOLINTERFACE_H_
