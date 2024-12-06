package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.DriveChassis;
import org.firstinspires.ftc.teamcode.utils.Numbers;
import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.PowerCurve;

@TeleOp(name="Autonomous Teleop", group="Autonomous")
public class AutoTeleop extends AutoOpMode {
    DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    @Override
    public void start() {
        leftFrontMotor = getHardware(DcMotorEx.class, "leftFront");
        leftBackMotor = getHardware(DcMotorEx.class, "leftRear");
        rightFrontMotor = getHardware(DcMotorEx.class, "rightFront");
        rightBackMotor = getHardware(DcMotorEx.class, "rightRear");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private double maxSpeed = 50;
    private final static float HORIZONTAL_BALANCE = 1.1f;
    private final double SPEED_CHANGE_PER_PRESS = 5;

    public double yaw;
    public double yawRad;
    public double normalizedYaw;
    public double targetRotation;
    @Override
    public void update() {
        yaw = orientation.getYaw();
        yawRad = orientation.getYaw(AngleUnit.RADIANS);
        normalizedYaw = Numbers.normalizeAngle(yaw);

        float moveXInput = controller1.axis(Controller.Axis.LeftStickX, PowerCurve.Quadratic);
        float moveYInput = controller1.axis(Controller.Axis.LeftStickY, PowerCurve.Quadratic);
        telemetry.addData("Controller X", moveXInput);
        telemetry.addData("Controller Y", moveYInput);
        float rotationInput = controller1.axis(Controller.Axis.RightStickX, PowerCurve.Cubic);

        if (controller1.pressed(Controller.Button.RightBumper))
            maxSpeed += SPEED_CHANGE_PER_PRESS;

        if (controller1.pressed(Controller.Button.LeftBumper))
            maxSpeed -= SPEED_CHANGE_PER_PRESS;
        maxSpeed = Range.clip(maxSpeed, 5, 90);
        if (controller1.pressed(Controller.Button.Back)) {
            imu.resetYaw();
            targetRotation = 0;
        }

        double verticalMovePower = moveXInput * Math.sin(-yawRad) + moveYInput * Math.cos(-yawRad);
        double horizontalMovePower = moveXInput * Math.cos(-yawRad) - moveYInput * Math.sin(-yawRad);
        horizontalMovePower *= HORIZONTAL_BALANCE;
        if (controller1.stoppedChanging(Controller.Axis.RightStickX))
            targetRotation = normalizedYaw;
        double turnPower;
        if (rotationInput != 0) turnPower = rotationInput;
        else turnPower = Numbers.turnCorrectionSpeed(normalizedYaw, targetRotation);

        double denominator = Math.max(Math.abs(verticalMovePower) + Math.abs(horizontalMovePower) + Math.abs(turnPower), 1);

        double leftFPower = (verticalMovePower + horizontalMovePower + turnPower) / denominator;
        double leftBPower = (verticalMovePower - horizontalMovePower + turnPower) / denominator;
        double rightFPower = (verticalMovePower - horizontalMovePower - turnPower) / denominator;
        double rightBPower = (verticalMovePower + horizontalMovePower - turnPower) / denominator;

        double velocityScale = DriveChassis.DRIVE_GEAR_RATIO * DriveChassis.TICKS_PER_REVOLUTION * maxSpeed / DriveChassis.WHEEL_CIRCUMFERENCE;

        leftFrontMotor.setVelocity(leftFPower * velocityScale);
        rightFrontMotor.setVelocity(rightFPower * velocityScale);
        leftBackMotor.setVelocity(leftBPower * velocityScale);
        rightBackMotor.setVelocity(rightBPower * velocityScale);

        telemetry.update();
    }
}
