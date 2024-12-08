package org.firstinspires.ftc.teamcode.xendy;

import java.io.Serializable;

public class SaveState implements Serializable {
    public double t;
    public double mX, mY, yaw, mS, turnPower;
    SaveState(double horzPow, double vertPow, double cYaw, double time, double maxSpeed, double turnPow) {
        t = time;
        mX = horzPow;
        mY = vertPow;
        yaw = cYaw;
        mS = maxSpeed;
        turnPower = turnPow;
    }
}