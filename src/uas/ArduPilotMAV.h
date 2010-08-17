#ifndef ARDUPILOTMAV_H
#define ARDUPILOTMAV_H

#include "UAS.h"

class ArduPilotMAV : public UAS
{
    Q_OBJECT
public:
    ArduPilotMAV(int id = 0);
public slots:
    /** @brief Receive a MAVLink message from this MAV */
    virtual void handleMessage(const mavlink_message_t& message);

};

#endif // ARDUPILOTMAV_H
