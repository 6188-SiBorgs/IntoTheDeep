package org.firstinspires.ftc.teamcode.xendy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="second chassis testing")
public class SecondChassisTesting extends OpMode {
    XDriveChassis chassis;
    @Override
    public void init() {
        chassis = new XDriveChassis(this);
    }

    @Override
    public void loop() {
        chassis.leftBackMotor.setPower(gamepad1.x ? 1 : 0);
        chassis.leftFrontMotor.setPower(gamepad1.y ? 1 : 0);
        chassis.rightBackMotor.setPower(gamepad1.a ? 1 : 0);
        chassis.rightFrontMotor.setPower(gamepad1.b ? 1 : 0);
    }
}
