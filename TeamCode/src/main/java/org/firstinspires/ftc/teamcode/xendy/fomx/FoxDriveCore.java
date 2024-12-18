package org.firstinspires.ftc.teamcode.xendy.fomx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class FoxDriveCore {
    public VisionPortal vision;
    public AprilTagProcessor aprilTag;
    public LinearOpMode opMode;
    public MultipleTelemetry telemetry;
    private IMU imu;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public FoxDriveCore(WebcamName webcam, IMU imu, LinearOpMode opMode) {
        this.imu = imu;
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), opMode.telemetry);
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        vision = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();
        telemetry.update();
        while (!opMode.isStopRequested() && (vision.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(10);
        }
        setManualExposure(6, 250);
    }


    double imuYaw;
    public void update() {
        imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addLine("Position calculation telemetry:");
        List<AprilTagDetection> tags = aprilTag.getDetections();
        for (AprilTagDetection tag : tags) {
            if (tag.metadata != null) {
                Pose3D robotPose = tag.robotPose;
                Position pos = robotPose.getPosition().toUnit(DistanceUnit.CM);
                double x = pos.x;
                double y = pos.y;
                double yaw = robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addLine("==============================");
                telemetry.addLine(String.format("From tag %s (id: %s)", tag.metadata.name, tag.id));
                telemetry.addLine(String.format("Robot Position: (%s, %s)", x, y));
                telemetry.addLine(String.format("Robot Yaw: %s (vs actual: %s)", yaw, imuYaw));
            } else {
                telemetry.addLine(String.format("Warning: found unknown tag %s!", tag.id));
            }
        }
        telemetry.update();
    }

    public void setManualExposure(int exposureMS, int gain) {
        ExposureControl exposureControl = vision.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = vision.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
    }

    public void sleep(long milliseconds) {
        opMode.sleep(milliseconds);
    }
}
