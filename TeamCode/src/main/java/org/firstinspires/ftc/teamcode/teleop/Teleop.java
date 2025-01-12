package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utils.controller.Controller.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.DriveChassis;
import org.firstinspires.ftc.teamcode.utils.Numbers;
import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;
import org.firstinspires.ftc.teamcode.utils.controller.PowerCurve;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private DriveChassis chassis;
    private double maxSpeed = 50;
    private final static float HORIZONTAL_BALANCE = 1.1f;

    private YawPitchRollAngles orientation;
    private double yaw;
    private double yawRad;
    private double normalizedYaw;

    private double targetRotation;
    private boolean lockAscention = false;

    // Gamepad States
    private final GameController controller1 = new GameController();
    private final GameController controller2 = new GameController();

    private final double SPEED_CHANGE_PER_PRESS = 5;

    @Override
    public void init() {
        chassis = new DriveChassis(this);
    }

    private boolean clawClosed = false;
    private boolean bucketDown = true;

    boolean firstRun = true;
    @Override
    public void loop() {
        if (firstRun) {
            chassis.collectionArmMotor.setTargetPosition(-250);
            chassis.collectionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            firstRun = false;
        }
        // Keeping track of the buttons from the last loop iteration so we do not need a billion booleans
        controller1.update(gamepad1);
        controller2.update(gamepad2);

        // Update the orientation of the robot each loop
        orientation = chassis.imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();
        yawRad = orientation.getYaw(AngleUnit.RADIANS);
        normalizedYaw = Numbers.normalizeAngle(yaw);

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Yaw Rad", yawRad);
        telemetry.addData("Normalized Yaw", normalizedYaw);

        float moveXInput = controller1.axis(Axis.LeftStickX, PowerCurve.Quadratic);
        float moveYInput = controller1.axis(Axis.LeftStickY, PowerCurve.Quadratic);
        float armXInput = -controller2.axis(Axis.RightStickX, PowerCurve.Cubic);
        float armYInput = -controller2.axis(Axis.LeftStickY, PowerCurve.Cubic);
        if (chassis.collectionArmMotor.getCurrentPosition() <= -2360 && armXInput < 0 || chassis.collectionArmMotor.getCurrentPosition() >= 0 && armXInput > 0) {
            chassis.collectionArmMotor.setVelocity(0);
            telemetry.addLine("======== ALERT ========");
            telemetry.addLine("LIMIT REACHED FOR COLLECTION ARM");
        }
        else {
            chassis.collectionArmMotor.setVelocity(armXInput * 500);
        }

        if (chassis.collectionArmMotor.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
            telemetry.addLine("Moving scoring arm position to extended!");
            telemetry.addLine("Cannot move vertical arm!");
            if (!chassis.collectionArmMotor.isBusy()) {
                chassis.collectionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        else {
            if (chassis.scoringArmMotor.getCurrentPosition() >= 0 && armYInput > 0 || chassis.scoringArmMotor.getCurrentPosition() <= -3100 && armYInput < 0) {
                chassis.scoringArmMotor.setVelocity(0);
                telemetry.addLine("======== ALERT ========");
                telemetry.addLine("LIMIT REACHED FOR SCORING ARM");
            }
            else chassis.scoringArmMotor.setVelocity(armYInput * 1150);
            double samv = chassis.scoringArmMotor.getVelocity();
            if (samv > 0 && chassis.scoringArmMotor.getCurrentPosition() > -800) {
                double mult = (Math.pow(chassis.scoringArmMotor.getCurrentPosition()/895.0, 2) + 0.2);
                if (mult > 1) mult = 1;
                chassis.scoringArmMotor.setVelocity(samv * mult);
                telemetry.addLine("Scoring arm " + Math.round(mult*100)/100 + "x slowdown");
            }
        }
        double ascension = -controller1.axis(Axis.LeftTrigger) + controller1.axis(Axis.RightTrigger);
        chassis.ascention.setPower(ascension);
        if (gamepad1.dpad_up) lockAscention = true;
        if (gamepad1.dpad_down) lockAscention = false;
        if (lockAscention) chassis.ascention.setPower(1);

        double pivot = (controller2.button(Controller.Button.DPadUp) ? 1 : 0) + (controller2.button(Controller.Button.DPadDown) ? -1 : 0);
        if (pivot < 0) chassis.endPivotMotor.setTargetPosition(-220);
        else chassis.endPivotMotor.setTargetPosition(0);
        chassis.endPivotMotor.setVelocity(pivot * 150);

        if (controller2.pressed(Controller.Button.A)) {
            chassis.claw.setPosition(clawClosed ? 1 : 0);
            clawClosed = !clawClosed;
        }

        if (controller2.pressed(Controller.Button.B)) {
            chassis.bucket.setPosition(bucketDown ? 1 : 0);
            bucketDown = !bucketDown;
        }

        float rotationInput = controller1.axis(Axis.RightStickX, PowerCurve.Cubic);

        if (controller1.pressed(Button.RightBumper))
            maxSpeed += SPEED_CHANGE_PER_PRESS;

        if (controller1.pressed(Button.LeftBumper))
            maxSpeed -= SPEED_CHANGE_PER_PRESS;

        maxSpeed = Range.clip(maxSpeed, 5, 90);

        telemetry.addData("Speed", maxSpeed);

        if (controller1.pressed(Button.Back)) {
            chassis.imu.resetYaw();
            targetRotation = 0;
        }

        // Thanks gm0!
        double verticalMovePower = moveXInput * Math.sin(-yawRad) + moveYInput * Math.cos(-yawRad);
        double horizontalMovePower = moveXInput * Math.cos(-yawRad) - moveYInput * Math.sin(-yawRad);

        // Correct for the imperfect strafing
        horizontalMovePower *= HORIZONTAL_BALANCE;

        if (controller1.stoppedChanging(Axis.RightStickX))
            targetRotation = normalizedYaw;

        double turnPower;
        if (rotationInput != 0) turnPower = rotationInput;
        else turnPower = Numbers.turnCorrectionSpeed(normalizedYaw, targetRotation);
        telemetry.addData("Rotation Input", rotationInput);
        telemetry.addData("Turn Power", turnPower);

        // Find maximum speed, to scale all of the motor powers to [-1, 1]
        double denominator = Math.max(Math.abs(verticalMovePower) + Math.abs(horizontalMovePower) + Math.abs(turnPower), 1);

        // Scale motor powers to [-1, 1]
        double leftFPower = (verticalMovePower + horizontalMovePower + turnPower) / denominator;
        double leftBPower = (verticalMovePower - horizontalMovePower + turnPower) / denominator;
        double rightFPower = (verticalMovePower - horizontalMovePower - turnPower) / denominator;
        double rightBPower = (verticalMovePower + horizontalMovePower - turnPower) / denominator;

        double velocityScale = chassis.DRIVE_GEAR_RATIO * chassis.TICKS_PER_REVOLUTION * maxSpeed / chassis.WHEEL_CIRCUMFERENCE;

        chassis.leftFrontMotor.setVelocity(leftFPower * velocityScale);
        chassis.rightFrontMotor.setVelocity(rightFPower * velocityScale);
        chassis.leftBackMotor.setVelocity(leftBPower * velocityScale);
        chassis.rightBackMotor.setVelocity(rightBPower * velocityScale);
//        telemetry.update();
    }
}
