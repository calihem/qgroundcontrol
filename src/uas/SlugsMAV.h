#ifndef SLUGSMAV_H
#define SLUGSMAV_H

#include "UAS.h"

class SlugsMAV : public UAS
{
    Q_OBJECT
    Q_INTERFACES(UASInterface)
public:
    SlugsMAV(int id = 0);

public slots:
    /** @brief Receive a MAVLink message from this MAV */
    virtual void handleMessage(const mavlink_message_t& message);
};

#endif // SLUGSMAV_H
