package org.firstinspires.ftc.teamcode.xendy;

import static org.firstinspires.ftc.teamcode.xendy.AutoUtils.error;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;

@TeleOp
public class SetPrimaryAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtils.update();

        if (!AutoUtils.error.isEmpty()) {
            telemetry.addLine(error);
            telemetry.update();
            waitForStart();
        }
        telemetry.addData("Currently selected binding", AutoUtils.selected);
        telemetry.addData("Path name:", AutoUtils.getSelectedName());
        telemetry.update();
        waitForStart();
        String binding = "";
        GameController controller1 = new GameController(gamepad1);
        while (binding.isEmpty() && opModeIsActive()) {
            telemetry.addLine("Please select a binding:");
            if (!AutoUtils.G1A.isEmpty()) telemetry.addData("A", AutoUtils.G1A);
            if (!AutoUtils.G1B.isEmpty()) telemetry.addData("B", AutoUtils.G1B);
            if (!AutoUtils.G1X.isEmpty()) telemetry.addData("X", AutoUtils.G1X);
            if (!AutoUtils.G1Y.isEmpty()) telemetry.addData("Y", AutoUtils.G1Y);
            if (!AutoUtils.G1DpadUp.isEmpty()) telemetry.addData("DpadUp", AutoUtils.G1DpadUp);
            if (!AutoUtils.G1DpadLeft.isEmpty()) telemetry.addData("DpadLeft", AutoUtils.G1DpadLeft);
            if (!AutoUtils.G1DpadRight.isEmpty()) telemetry.addData("DpadRight", AutoUtils.G1DpadRight);
            if (!AutoUtils.G1DpadDown.isEmpty()) telemetry.addData("DpadDown", AutoUtils.G1DpadDown);
            controller1.update(gamepad1);
            if (controller1.pressed(Controller.Button.A)) binding = "G1A";
            else if (controller1.pressed(Controller.Button.B)) binding = "G1B";
            else if (controller1.pressed(Controller.Button.X)) binding = "G1X";
            else if (controller1.pressed(Controller.Button.Y)) binding = "G1Y";
            else if (controller1.pressed(Controller.Button.DPadUp)) binding = "G1DpadUp";
            else if (controller1.pressed(Controller.Button.DPadDown)) binding = "G1DpadDown";
            else if (controller1.pressed(Controller.Button.DPadRight)) binding = "G1DpadRight";
            else if (controller1.pressed(Controller.Button.DPadLeft)) binding = "G1DpadLeft";
            telemetry.update();
        }
        if (opModeIsActive()) {
            AutoUtils.setPrimary(binding);
            telemetry.addData("Set selected binding to", binding);
            telemetry.addData("PathName", AutoUtils.getSelectedName());
            telemetry.update();
            while (opModeIsActive()) {
            }
        }
    }
}
