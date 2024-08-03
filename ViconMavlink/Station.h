/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2017 by University of Illinois			   *
 *                                                                         *
 *   http://illinois.edu                                                   *
 *                                                                         *
 ***************************************************************************/

/**
 * @file    Station.h
 *
 * A Station object organizes drones and senders, communicates with Vicon and
 * updates measurements for drones using a separate thread.
 *
 * A StationWindow is the main GUI.
 *
 * This is the Model of the
 * Model(Station)-View(StationWindow)-Controller(StationController) pattern.
 *
 * @author  Bo Liu  <boliu1@illinois.edu>
 *
 */
#ifndef STATION_H
#define STATION_H

#include <QObject>
#include <QString>
#include <QHostAddress>
#include <QMap>
#include <QList>
#include "DataStreamClient.h"
#include <iostream>
#include "Drone.h"
#include <QReadWriteLock>
#include <QtConcurrent/QtConcurrent>

class Station : public QObject
{
    Q_OBJECT
public:
    explicit Station(QObject *parent = 0);
    virtual ~Station();

    QHostAddress getHostAddress() const;
    void setHostAddress(const QHostAddress &value);

    quint16 getHostPort() const;
    void setHostPort(const quint16 &value);

    double getRate() const;
    void setRate(double value);

    void connectVicon();
    void disconnectVicon();

    void addDrone(const QString& name);

    void removeDrone(const QString& name);

    void dataStream();

    QMap<QString, Drone> droneCollection;
    QReadWriteLock lock;

    void setNorth(const QString &axis);

    double getdt() const;
    void setdt(double value);

    long long getFrame() const;
    void setFrame(long long value);

    DroneState getState(const QString &name, uint64_t &time_boot_us);

signals:
    void ViconConnected();
    void ViconDisconnected();
    void droneAdded(QString name);
    void droneRemoved(QString name);
    void dtUpdated(double dt);

public slots:

private:
    double dt; // seconds/frame
    long long frame;
    double rate;

    ViconDataStreamSDK::CPP::Client vicon_;
    bool shouldExit;
    bool isInitialized;

    QHostAddress hostAddress;
    quint16 hostPort;

    QString north;

    QFuture<void> dataStreamFuture;
    QFuture<void> viconConnectFuture;
    void _connectVicon();
    void _dataStream();
    void setupConnections();
    void initialize();

    // helpers
    QString _Adapt( const ViconDataStreamSDK::CPP::Direction::Enum i_Direction );

};

#endif // STATION_H
