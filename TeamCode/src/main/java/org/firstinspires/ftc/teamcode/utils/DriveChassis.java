package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class with all of the common data about our robot chassis so we can reuse it in multiple OpModes without reusing too much code
 */
public class DriveChassis {
    // Initialize our motors
    public DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    // -50 to -2150
    public DcMotorEx collectionArmMotor, scoringArmMotor, endPivotMotor, ascention;
    public Servo bucket;
    public Servo claw;

    // Info about our robots design
    public static final int TICKS_PER_REVOLUTION = 28;
    public static final double DRIVE_GEAR_RATIO = 20;
    public static final double WHEEL_CIRCUMFERENCE = 23.94; // In CM
    public final double ARM_GEAR_RATIO = 60;

    public IMU imu;

    public DriveChassis(OpMode opMode) {
        this(opMode, true);
    }

    public DriveChassis(OpMode opMode, boolean includeArms) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Instantiate each motor
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (includeArms) {
            // Other motors (Arms, end effectors, etc)
            collectionArmMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "horizontalSlide");
            scoringArmMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "verticalSlide");
            endPivotMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "endPivotMotor");
            ascention = (DcMotorEx) hardwareMap.get(DcMotor.class, "ascension");

            scoringArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            collectionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            endPivotMotor.setTargetPosition(0);
            endPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ascention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            scoringArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            collectionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            endPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            claw = hardwareMap.get(Servo.class, "intakeEffector");
            bucket = hardwareMap.get(Servo.class, "bucket");
        }

        // Instantiate the imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                ))
        );
        imu.resetYaw();
    }
}
