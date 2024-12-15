package org.firstinspires.ftc.teamcode.xendy;

import java.io.Serializable;

public class SaveState implements Serializable {
    public double t;
    public double mX, mY, yaw, mS, turnPower;

    // Horizontal Arm
    public double horizArmPosition;
    public double vertArmPosition;
    public double clawPosition;
    public double bucketPosition;
    SaveState(double horzPow, double vertPow, double cYaw, double time, double maxSpeed, double turnPow, double harmpos, double varmpos, double bucket, double claw) {
        t = time;
        mX = horzPow;
        mY = vertPow;
        yaw = cYaw;
        mS = maxSpeed;
        turnPower = turnPow;

        horizArmPosition = harmpos;
        vertArmPosition = varmpos;
        bucketPosition = bucket;
        clawPosition = claw;
    }
}