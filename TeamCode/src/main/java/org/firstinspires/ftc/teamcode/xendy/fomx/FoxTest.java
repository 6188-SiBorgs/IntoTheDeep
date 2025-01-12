package org.firstinspires.ftc.teamcode.xendy.fomx;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Disabled
@TeleOp(name = "FoxTest")
public class FoxTest extends LinearOpMode {
    FoxDriveCore foxDriveCore;
    private IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                ))
        );
        imu.resetYaw();
        FoxDriveCore foxDriveCore = new FoxDriveCore(
                hardwareMap.get(WebcamName.class, "Webcam"),
                imu,
                this
        );

        while (opModeInInit()) {

        }

        while (opModeIsActive())
        {
            foxDriveCore.update();
        }
    }
}
