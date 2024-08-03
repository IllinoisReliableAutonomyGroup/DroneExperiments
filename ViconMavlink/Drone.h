/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2017 by University of Illinois			   *
 *                                                                         *
 *   http://illinois.edu                                                   *
 *                                                                         *
 ***************************************************************************/

/**
 * @file    Drone.h
 *
 * A Drone object stores raw measurements fetched from Vicon.
 *
 * @author  Bo Liu  <boliu1@illinois.edu>
 *
 */

#ifndef DRONE_H
#define DRONE_H

#include <QString>
#include "lib/MAVLink2/common/mavlink.h"
#include "DroneState.h"

class Drone
{
public:
    Drone();
    virtual ~Drone();
    Drone(QString droneName, double _dt, const DroneState &_state);
    Drone(QString droneName, double _dt);
    QString getName() const;
    void setName(const QString &value);

    DroneState getState(uint64_t &time_boot_us) const;
    void updateState(const mavlink_att_pos_mocap_t &att_pos, long long _frame);

    void reset();

    long long getTime() const;

    void setGamma(double value);
    void setdt(double value);

private:
    QString name;
    DroneState state;
    long long frame;
    double dt;
    double gamma = 0.7;
    bool isInitialized = false;
};

#endif // DRONE_H
