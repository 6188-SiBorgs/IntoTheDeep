package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.xendy.DriveChassisX;

@Disabled
@TeleOp(name="second chassis testing")
public class SecondChassisTesting extends OpMode {
    DriveChassisX chassis;
    @Override
    public void init() {
        chassis = new DriveChassisX(this);
    }

    @Override
    public void loop() {
        chassis.leftBackMotor.setPower(gamepad1.x ? 1 : 0);
        chassis.leftFrontMotor.setPower(gamepad1.y ? 1 : 0);
        chassis.rightBackMotor.setPower(gamepad1.a ? 1 : 0);
        chassis.rightFrontMotor.setPower(gamepad1.b ? 1 : 0);
    }
}
