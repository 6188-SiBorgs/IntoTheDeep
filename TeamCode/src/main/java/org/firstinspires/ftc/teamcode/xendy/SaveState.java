package org.firstinspires.ftc.teamcode.xendy;

import java.io.Serializable;

public class SaveState implements Serializable {
    public double t;
    public double mX, mY, yaw, mS, turnPower;

    // Horizontal Arm
    public int horizArmPosition;
    public int vertArmPosition;
    public double clawPosition;
    public double bucketPosition;

    public double v;
    SaveState(double horzPow, double vertPow, double cYaw, double time, double maxSpeed, double turnPow, int harmpos, int varmpos, double bucket, double claw, double voltage) {
        t = time;
        mX = horzPow;
        mY = vertPow;
        yaw = cYaw;
        mS = maxSpeed;
        turnPower = turnPow;
        v = voltage;
        horizArmPosition = harmpos;
        vertArmPosition = varmpos;
        bucketPosition = bucket;
        clawPosition = claw;
    }
}