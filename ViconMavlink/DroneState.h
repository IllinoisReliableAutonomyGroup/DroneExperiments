#ifndef DRONESTATE_H
#define DRONESTATE_H

struct DroneState{
    double x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate, qx, qy, qz, qw;
    void reset(){
        x = 0;
        y = 0;
        z = 0;
        vx = 0;
        vy = 0;
        vz = 0;
        ax = 0;
        ay = 0;
        az = 0;
        yaw = 0;
        yaw_rate = 0;
        qx = 0;
        qy = 0;
        qz = 0;
        qw = 1;
    }
    DroneState(){
        reset();
    }
};

#endif // DRONESTATE_H
