package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;

import java.util.HashMap;

public abstract class AutoOpMode extends OpMode {
    public abstract void start();
    public abstract void update();


    public boolean isAutonomous = false;
    public final GameController controller1 = new GameController();
    public final GameController controller2 = new GameController();
    public IMU imu;
    public YawPitchRollAngles orientation;

    HashMap<String, DcMotorEx> DcMotorMap = new HashMap<>();

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ))
        );
        imu.resetYaw();
        start();
    }

    @Override
    public void loop() {
        controller1.update(gamepad1);
        controller2.update(gamepad2);
        orientation = imu.getRobotYawPitchRollAngles();
        update();
    }

    public <T> T getHardware(Class<? extends T> classOrInterface, String deviceName) {
        T hardware = hardwareMap.get(classOrInterface, deviceName);
        if (classOrInterface.equals(DcMotorEx.class)) {
            DcMotorMap.put(deviceName, (DcMotorEx) hardware);
        }
        return hardware;
    }
}
