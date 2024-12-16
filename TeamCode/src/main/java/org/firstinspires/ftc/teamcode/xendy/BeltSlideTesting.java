package org.firstinspires.ftc.teamcode.xendy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.DriveChassis;
@TeleOp(name="BeltSlideTesting", group="z_testing")
public class BeltSlideTesting extends OpMode {
    DriveChassis chassis;
    @Override
    public void init() {
        chassis = new DriveChassis(this);
    }

    @Override
    public void loop() {
        chassis.scoringArmMotor.setVelocity(1000 * gamepad1.left_stick_y);
        chassis.collectionArmMotor.setVelocity(1000 * gamepad1.right_stick_x);
        double epm = ((gamepad1.dpad_up ? 1 : 0) + (gamepad1.dpad_down ? -1 : 0));
        chassis.endPivotMotor.setVelocity(epm * 300);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("pos", chassis.scoringArmMotor.getCurrentPosition());
        telemetry.addData("ly", gamepad1.right_stick_x);
        telemetry.addData("pos", chassis.collectionArmMotor.getCurrentPosition());
        telemetry.addData("endPivot", epm);
        telemetry.addData("pos", chassis.endPivotMotor.getCurrentPosition());

        if (gamepad1.a) {
            chassis.scoringArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chassis.scoringArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.b) {
            chassis.collectionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chassis.collectionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.x) {
            chassis.endPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chassis.endPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.update();
    }
}
