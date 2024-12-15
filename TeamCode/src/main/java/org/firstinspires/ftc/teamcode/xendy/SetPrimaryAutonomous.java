package org.firstinspires.ftc.teamcode.xendy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;

@TeleOp
public class SetPrimaryAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousSwitcherHelpers.update();
        telemetry.addData("Currently selected binding", AutonomousSwitcherHelpers.selected);
        telemetry.addData("Path name:", AutonomousSwitcherHelpers.getSelectedName());
        telemetry.update();
        waitForStart();
        String binding = "";
        GameController controller1 = new GameController(gamepad1);
        while (binding.isEmpty() && opModeIsActive()) {
            telemetry.addLine("Please select a binding:");
            if (!AutonomousSwitcherHelpers.G1A.isEmpty()) telemetry.addData("A", AutonomousSwitcherHelpers.G1A);
            if (!AutonomousSwitcherHelpers.G1B.isEmpty()) telemetry.addData("B", AutonomousSwitcherHelpers.G1B);
            if (!AutonomousSwitcherHelpers.G1X.isEmpty()) telemetry.addData("X", AutonomousSwitcherHelpers.G1X);
            if (!AutonomousSwitcherHelpers.G1Y.isEmpty()) telemetry.addData("Y", AutonomousSwitcherHelpers.G1Y);
            if (!AutonomousSwitcherHelpers.G1DpadUp.isEmpty()) telemetry.addData("DpadUp", AutonomousSwitcherHelpers.G1DpadUp);
            if (!AutonomousSwitcherHelpers.G1DpadLeft.isEmpty()) telemetry.addData("DpadLeft", AutonomousSwitcherHelpers.G1DpadLeft);
            if (!AutonomousSwitcherHelpers.G1DpadRight.isEmpty()) telemetry.addData("DpadRight", AutonomousSwitcherHelpers.G1DpadRight);
            if (!AutonomousSwitcherHelpers.G1DpadDown.isEmpty()) telemetry.addData("DpadDown", AutonomousSwitcherHelpers.G1DpadDown);
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
            AutonomousSwitcherHelpers.setPrimary(binding);
            telemetry.addData("Set selected binding to", binding);
            telemetry.addData("PathName", AutonomousSwitcherHelpers.getSelectedName());
            telemetry.update();
            while (opModeIsActive()) {
            }
        }
    }
}
