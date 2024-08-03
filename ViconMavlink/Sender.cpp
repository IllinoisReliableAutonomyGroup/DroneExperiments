/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2017 by University of Illinois			   *
 *                                                                         *
 *   http://illinois.edu                                                   *
 *                                                                         *
 ***************************************************************************/

/**
 * @file    Sender.cpp
 *
 * A Sender object reads Vicon data from a Station object, processes the data
 * with Kalman filtering and computes a GPS MAVLink data pack, sends MAVLink
 * messages to a robot via UDP.
 *
 * This is the Model of the
 * Model(Sender)-View(SenderWindow)-Controller(SenderController) pattern.
 *
 * @author  Bo Liu  <boliu1@illinois.edu>
 *
 */
#include "Sender.h"
#include <cmath>
#include <chrono>

Sender::Sender(const QString &name, std::unique_ptr<Station> &station, QObject *parent) : QObject(parent),
    isInitialized {false},
    isRunning {false},
    station {station},
    name {name},
    udpSocket { new QUdpSocket(this)}
{
    initialize();
    setupConnections();
}

Sender::~Sender()
{
    delete udpSocket;
    qDebug() << "Sender destructor.";
}

void Sender::setupConnections()
{
    connect(&timer, SIGNAL(timeout()),
	    this, SLOT(timerHandler()));
}

void Sender::start()
{
    startTimer(rate);
}

void Sender::stop()
{
    stopTimer();
}

void Sender::initialize()
{
    /* the following will appear on UI */
    rate = 30;
    sysid = 1;
    compid = 1;
    remoteAddress = QHostAddress("192.168.1.22");
    remotePort = 10086;
}

void Sender::startTimer(uint8_t rate)
{
    auto msecTime = 1.0 / rate * 1000;
    timer.start(msecTime);
    isRunning = true;
}

void Sender::stopTimer()
{
    timer.stop();
    isRunning = false;
}

void Sender::updateTimer(uint8_t r)
{
    rate = r;
    if (isRunning) {
        stopTimer();
        startTimer(rate);
    }
}

QHostAddress Sender::getRemoteAddress() const
{
    return remoteAddress;
}

quint16 Sender::getRemotePort() const
{
    return remotePort;
}

uint8_t Sender::getSysID() const
{
    return sysid;
}

uint8_t Sender::getCompID() const
{
    return compid;
}

uint8_t Sender::getRate() const
{
    return rate;
}

void Sender::setRate(uint8_t rate)
{
    this->rate = rate;
    updateTimer(rate);
}

void Sender::setSysID(uint8_t id)
{
    sysid = id;
}

void Sender::setCompID(uint8_t id)
{
    compid = id;
}

void Sender::timerHandler()
{
    sendDatagram();
    emit measUpdated();
}

void Sender::setRemoteAddress(const QString &value)
{
    remoteAddress = QHostAddress(value);
}

void Sender::setRemotePort(quint16 port)
{
    remotePort = port;
}

void Sender::sendDatagram()
{
    uint8_t len = 0;
    constexpr int BUFSIZE = 512;

	qDebug() << "sending locPos";
	mavlink_message_t msg_loc;
	uint8_t send_buf_loc[BUFSIZE];
	memset(send_buf_loc, 0, BUFSIZE);

    uint64_t time_boot_us;
    DroneState state = station->getState(name, time_boot_us);
    locPosMsg.time_usec = time_boot_us;
    locPosMsg.x = state.x;
    locPosMsg.y = state.y;
    locPosMsg.z = state.z;
    locPosMsg.vx = state.vx;
    locPosMsg.vy = state.vy;
    locPosMsg.vz = state.vz;
    locPosMsg.ax = state.ax;
    locPosMsg.ay = state.ay;
    locPosMsg.az = state.az;
    // adding offset to make cov positive definite. cov[0-8] are the diag.
    double offset = 100;
    locPosMsg.covariance[0] = offset + state.yaw;
    locPosMsg.covariance[1] = offset + state.yaw_rate;
    locPosMsg.covariance[2] = offset + state.qx;
    locPosMsg.covariance[3] = offset + state.qy;
    locPosMsg.covariance[4] = offset + state.qz;
    locPosMsg.covariance[5] = offset + state.qw;
    locPosMsg.covariance[6] = offset;
    locPosMsg.covariance[7] = offset;
    locPosMsg.covariance[8] = offset;
    mavlink_msg_local_position_ned_cov_encode(sysid, compid, &msg_loc, &locPosMsg);
    len = mavlink_msg_to_send_buffer(send_buf_loc, &msg_loc);
	qDebug() << "sending to " << remoteAddress << "," << remotePort;
	udpSocket->writeDatagram(reinterpret_cast<const char*>(send_buf_loc), len, remoteAddress, remotePort);
}
