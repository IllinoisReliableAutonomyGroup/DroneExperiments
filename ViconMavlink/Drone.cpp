/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2017 by University of Illinois			   *
 *                                                                         *
 *   http://illinois.edu                                                   *
 *                                                                         *
 ***************************************************************************/

/**
 * @file    Drone.cpp
 *
 * A Drone object stores raw measurements fetched from Vicon.
 *
 * @author  Bo Liu  <boliu1@illinois.edu>
 *
 */
#include "Drone.h"
#include <QDebug>
#include "lib/Eigen/Geometry"

Drone::Drone()
{

}

Drone::Drone(QString droneName, double _dt)
{
    name = droneName;
    dt = _dt;
}

Drone::~Drone()
{
    // nothing to do
}

Drone::Drone(QString droneName, double _dt, const DroneState &_state)
{
    name = droneName;
    dt = _dt;
    state = _state;
}

QString Drone::getName() const
{
    return name;
}

void Drone::setName(const QString &value)
{
    name = value;
}

DroneState Drone::getState(uint64_t &time_boot_us) const
{
    time_boot_us = int(frame * dt * 1e6);
    return state;
}

void Drone::updateState(const mavlink_att_pos_mocap_t &att_pos, long long _frame){
    Eigen::Quaternionf q(att_pos.q[3], att_pos.q[0], att_pos.q[1], att_pos.q[2]);
    auto euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    double yaw = euler[0];

    qDebug() << att_pos.x << " " << att_pos.y << " " << att_pos.z << " " << euler[2] << " " << euler[1] << " " << euler[0];

    state.qx = att_pos.q[0];
    state.qy = att_pos.q[1];
    state.qz = att_pos.q[2];
    state.qw = att_pos.q[3];

    if(!isInitialized){
        state.x = att_pos.x;
        state.y = att_pos.y;
        state.z = att_pos.z;
        state.yaw = yaw;
        frame = _frame;
        isInitialized = true;
        return;
    }

//    qDebug() << "updateState. frame == " << _frame;
    if(_frame == frame) return;

    double delta_t = (_frame - frame) * dt;

    DroneState new_state;
    new_state.x = gamma * state.x + (1 - gamma) * att_pos.x;
    new_state.y = gamma * state.y + (1 - gamma) * att_pos.y;
    new_state.z = gamma * state.z + (1 - gamma) * att_pos.z;
    new_state.yaw = gamma * state.yaw + (1 - gamma) * yaw;

    new_state.vx = (new_state.x - state.x) / delta_t;
    new_state.vy = (new_state.y - state.y) / delta_t;
    new_state.vz = (new_state.z - state.z) / delta_t;
    new_state.yaw_rate = (new_state.yaw - state.yaw) / delta_t;
    new_state.vx = gamma * state.vx + (1 - gamma) * new_state.vx;
    new_state.vy = gamma * state.vy + (1 - gamma) * new_state.vy;
    new_state.vz = gamma * state.vz + (1 - gamma) * new_state.vz;
    new_state.yaw_rate = gamma * state.yaw_rate + (1 - gamma) * new_state.yaw_rate;

    new_state.ax = (new_state.vx - state.vx) / delta_t;
    new_state.ay = (new_state.vy - state.vy) / delta_t;
    new_state.az = (new_state.vz - state.vz) / delta_t;
    new_state.ax = gamma * state.ax + (1 - gamma) * new_state.ax;
    new_state.ay = gamma * state.ay + (1 - gamma) * new_state.ay;
    new_state.az = gamma * state.az + (1 - gamma) * new_state.az;

    state.x = new_state.x;
    state.y = new_state.y;
    state.z = new_state.z;
    state.yaw = new_state.yaw;
    state.vx = new_state.vx;
    state.vy = new_state.vy;
    state.vz = new_state.vz;
    state.yaw_rate = new_state.yaw_rate;
    state.ax = new_state.ax;
    state.ay = new_state.ay;
    state.az = new_state.az;

    frame = _frame;
}

void Drone::setGamma(double value){
    gamma = value;
}

void Drone::setdt(double value){
    dt = value;
    reset();
}

void Drone::reset()
{
    state.reset();
    frame = 0;
    isInitialized = false;
}

long long Drone::getTime() const
{
    return frame;
}
