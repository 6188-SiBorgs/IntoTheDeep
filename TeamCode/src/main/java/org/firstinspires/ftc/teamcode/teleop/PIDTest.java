package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.DriveChassis;
import org.firstinspires.ftc.teamcode.utils.Numbers;

@TeleOp(name = "PID Testing")
public class PIDTest extends LinearOpMode {

    private DriveChassis chassis;

    private double integralSum = 0;
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private YawPitchRollAngles orientation;
    private double yaw;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new DriveChassis(this, false);

        waitForStart();

        double targetAngle = 0;
        while (opModeIsActive()) {
            orientation = chassis.imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw(AngleUnit.RADIANS);

            if (gamepad1.dpad_up) {
                Kp += 0.1;
            } else if (gamepad1.dpad_down) {
                Kp -= 0.1;
            }

            if (gamepad1.dpad_right) {
                Ki += 0.1;
            } else if (gamepad1.dpad_left) {
                Ki -= 0.1;
            }

            if (gamepad1.right_bumper) {
                Kd += 0.1;
            } else if (gamepad1.left_bumper) {
                Kd -= 0.1;
            }

            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();

            double turnPower = PIDControl(targetAngle, yaw);
            chassis.leftFrontMotor.setPower(turnPower);
            chassis.leftBackMotor.setPower(turnPower);
            chassis.rightFrontMotor.setPower(-turnPower);
            chassis.rightBackMotor.setPower(-turnPower);
        }
    }

    public double PIDControl(double reference, double state) {
        double error = Numbers.wrapAngle(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (integralSum * Ki) + (derivative * Kd);
    }
}
