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
 *   @brief Definition of central factory class for all unmanned aerial systems
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#ifndef _UASFACTORY_H_
#define _UASFACTORY_H_

#include <QMutex>
#include <mavlink.h>
#include "UASInterface.h"
#include "UAS.h"
#include "SlugsMAV.h"
#include "PxQuadMAV.h"
#include "ArduPilotMAV.h"

/**
 * @brief Factory class for the UASs
 *
 **/
class UASFactory
{
	public:
		static UASInterface* buildUAV(MAV_AUTOPILOT_TYPE autopilotType, int id);

	private:
		UASFactory();
		~UASFactory();

};

inline UASInterface* UASFactory::buildUAV(MAV_AUTOPILOT_TYPE autopilotType, int id)
{
	UASInterface *uas = NULL;

	switch (autopilotType)
	{
		case MAV_AUTOPILOT_GENERIC:
			uas = new UAS(id);
			break;
		case MAV_AUTOPILOT_PIXHAWK:
			// Fixme differentiate between quadrotor and coaxial here
			uas = new PxQuadMAV(id);
			break;
		case MAV_AUTOPILOT_SLUGS:
			uas = new SlugsMAV(id);
			break;
		case MAV_AUTOPILOT_ARDUPILOT:
			uas = new ArduPilotMAV(id);
			break;
		default:
			break;
	}

	return uas;
}

#endif // _UASFACTORY_H_
