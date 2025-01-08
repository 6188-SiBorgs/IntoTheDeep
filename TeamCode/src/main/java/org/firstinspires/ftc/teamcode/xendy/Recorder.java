package org.firstinspires.ftc.teamcode.xendy;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Numbers;
import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;
import org.firstinspires.ftc.teamcode.utils.controller.PowerCurve;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

@Config
@TeleOp(name="Autonomous Recorder")
public class Recorder extends OpMode {
    public static String pathName = "";
    public String binding = "";
    private final double MAX_HESITATION_TIME = 0.3;

    private DriveChassisX chassis;
    private double maxSpeed = 50;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private YawPitchRollAngles orientation;
    private double yaw;
    private double yawRad;
    private double normalizedYaw;

    private double targetRotation;

    public static final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/XendysPessimisticAutonomous/";

    // Gamepad States
    private final GameController controller1 = new GameController();
    private final GameController controller2 = new GameController();

    private final double SPEED_CHANGE_PER_PRESS = 5;

    private ArrayList<SaveState> states = new ArrayList<>();
    private int index = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        chassis = new DriveChassisX(this);
        pathName = "";
    }

    private boolean DONE_RECORDING = false;
    private boolean SENT_TO_TELEMETRY = false;
    private boolean RECORDING = false;

    private double idleStartTime = -1;
    private double idleRemoveTime = 0;
    private boolean idling = false;

    // For arm, will be used
    private boolean initializing = true;
    private boolean firstinit = true;

    private boolean updatedAutoUtils = false;
    private boolean clawClosed = false;
    private boolean bucketDown = true;
    private String overrideMessage = "";

    @Override
    public void loop() {
        if (firstinit) {
            chassis.isAuto();
            chassis.scoringArmMotor.setVelocity(2000);
            chassis.endPivotMotor.setVelocity(2000);
            chassis.collectionArmMotor.setVelocity(2000);
        }
        if (initializing) {
            if (!(chassis.collectionArmMotor.isBusy() || chassis.scoringArmMotor.isBusy() || chassis.endPivotMotor.isBusy())) {
                initializing = false;
                chassis.scoringArmMotor.setVelocity(0);
                chassis.endPivotMotor.setVelocity(0);
                chassis.collectionArmMotor.setVelocity(0);
                chassis.goTele();
            }
            else {
                telemetry.addLine("Please wait as we reset arm motors...");
                telemetry.addData("Collection slide busy?", chassis.collectionArmMotor.isBusy());
                telemetry.addData("Scoring slide busy?", chassis.scoringArmMotor.isBusy());
                telemetry.addData("End pivot busy?", chassis.endPivotMotor.isBusy());
                telemetry.update();
                return;
            }
        }
        // TODO: Potentially use a state machine to reduce if/else nesting?
        controller1.update(gamepad1);
        controller2.update(gamepad2);
        if (controller1.pressed(Controller.Button.Start)) {
            DONE_RECORDING = true;
        }
        if (DONE_RECORDING) {
            if (!SENT_TO_TELEMETRY) {
                if (pathName.isEmpty()) {
                    telemetry.addLine("Please write and save the name in the dashboard");
                }
                else if (binding.isEmpty()) {
                    if (!updatedAutoUtils) {
                        updatedAutoUtils = true;
                        AutoUtils.update();
                    }
                    telemetry.addLine("Please choose a binding for this path. (You can override other paths)");
                    telemetry.addLine("(A/B/Y/X, Dpad)");
                    telemetry.addLine();
                    telemetry.addLine("Saved paths:");
                    if (!AutoUtils.G1A.isEmpty()) telemetry.addData("A", AutoUtils.G1A);
                    else telemetry.addData("A", "<open>");
                    if (!AutoUtils.G1B.isEmpty()) telemetry.addData("B", AutoUtils.G1B);
                    else telemetry.addData("B", "<open>");
                    if (!AutoUtils.G1X.isEmpty()) telemetry.addData("X", AutoUtils.G1X);
                    else telemetry.addData("X", "<open>");
                    if (!AutoUtils.G1Y.isEmpty()) telemetry.addData("Y", AutoUtils.G1Y);
                    else telemetry.addData("Y", "<open>");
                    if (!AutoUtils.G1DpadUp.isEmpty()) telemetry.addData("DpadUp", AutoUtils.G1DpadUp);
                    else telemetry.addData("DpadUp", "<open>");
                    if (!AutoUtils.G1DpadLeft.isEmpty()) telemetry.addData("DpadLeft", AutoUtils.G1DpadLeft);
                    else telemetry.addData("Dpadleft", "<open>");
                    if (!AutoUtils.G1DpadRight.isEmpty()) telemetry.addData("DpadRight", AutoUtils.G1DpadRight);
                    else telemetry.addData("Dpadright", "<open>");
                    if (!AutoUtils.G1DpadDown.isEmpty()) telemetry.addData("DpadDown", AutoUtils.G1DpadDown);
                    else telemetry.addData("Dpaddown", "<open>");

                    Controller.Button button = controller1.getMostRecentlyPressedButton();
                    switch (button) {
                        case A:
                            binding = "G1A";
                            overrideMessage = AutoUtils.G1A;
                            break;
                        case B:
                            binding = "G1B";
                            overrideMessage = AutoUtils.G1B;
                            break;
                        case X:
                            binding = "G1X";
                            overrideMessage = AutoUtils.G1X;
                            break;
                        case Y:
                            binding = "G1Y";
                            overrideMessage = AutoUtils.G1Y;
                            break;
                        case DPadUp:
                            binding = "G1DpadUp";
                            overrideMessage = AutoUtils.G1DpadUp;
                            break;
                        case DPadDown:
                            binding = "G1DpadDown";
                            overrideMessage = AutoUtils.G1DpadDown;
                            break;
                        case DPadLeft:
                            binding = "G1DpadLeft";
                            overrideMessage = AutoUtils.G1DpadLeft;
                            break;
                        case DPadRight:
                            binding = "G1DpadRight";
                            overrideMessage = AutoUtils.G1DpadRight;
                            break;
                    }
                }
                else if (!overrideMessage.isEmpty()) {
                    telemetry.addLine(String.format("Are you sure you want to override \"%s\"?", overrideMessage));
                    telemetry.addLine("A to confirm, B to go back.");
                    telemetry.update();
                    if (controller1.pressed(Controller.Button.A)) overrideMessage = "";
                    if (controller1.pressed(Controller.Button.B)) {
                        overrideMessage = "";
                        binding = "";
                    }
                }
                else {
                    try {
                        serializeStates(binding);
                    } catch (IOException e) {
                        telemetry.addLine(AutoUtils.formatIOException(e));
                        telemetry.update();
                    }
                    SENT_TO_TELEMETRY = true;
                }
            }
            chassis.leftBackMotor.setVelocity(0);
            chassis.leftFrontMotor.setVelocity(0);
            chassis.rightBackMotor.setVelocity(0);
            chassis.rightFrontMotor.setVelocity(0);
            return;
        }

        orientation = chassis.imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();
        telemetry.addData("Yaw", yaw);
        yawRad = orientation.getYaw(AngleUnit.RADIANS);
        normalizedYaw = Numbers.normalizeAngle(yaw);

        float moveXInput = controller1.axis(Controller.Axis.LeftStickX, PowerCurve.Quadratic);
        float moveYInput = controller1.axis(Controller.Axis.LeftStickY, PowerCurve.Quadratic);
        float rotationInput = controller1.axis(Controller.Axis.RightStickX, PowerCurve.Cubic);

        float armXInput = -controller2.axis(Controller.Axis.RightStickX, PowerCurve.Cubic);
        float armYInput = -controller2.axis(Controller.Axis.LeftStickY, PowerCurve.Cubic);
        boolean clawInput = controller2.pressed(Controller.Button.A);
        boolean bucketInput = controller2.pressed(Controller.Button.B);
        double pivot = (controller2.button(Controller.Button.DPadUp) ? 1 : 0) + (controller2.button(Controller.Button.DPadDown) ? -1 : 0);

        if (clawInput) {
            chassis.claw.setPosition(clawClosed ? 1 : 0);
            clawClosed = !clawClosed;
        }

        if (bucketInput) {
            chassis.bucket.setPosition(bucketDown ? 1 : 0);
            bucketDown = !bucketDown;
        }

        boolean not_inputed = moveXInput + moveYInput + rotationInput + armXInput + armYInput + pivot == 0 &&
                            !clawInput && !bucketInput;


        if (!not_inputed && !RECORDING) {
            RECORDING = true;
            resetRuntime();
            chassis.imu.resetYaw();
            yaw = 0;
            yawRad = 0;
            normalizedYaw = 0;
        }
        else {
            telemetry.addLine("Ready to record!");
            telemetry.addLine("Waiting for first input");
        }

        if (RECORDING && not_inputed) {
            // idleing
            if (idleStartTime == -1) {
                idleStartTime = getRuntime();
            }
            double idleTime = getRuntime() - idleStartTime;
            if (idleTime > MAX_HESITATION_TIME) {
                idling = true;
                idleRemoveTime = idleTime - MAX_HESITATION_TIME;
            }
        }
        else {
            idleStartTime = -1;
            idling = false;
        }

        if (controller1.pressed(Controller.Button.RightBumper))
            maxSpeed += SPEED_CHANGE_PER_PRESS;

        if (controller1.pressed(Controller.Button.LeftBumper))
            maxSpeed -= SPEED_CHANGE_PER_PRESS;
        maxSpeed = Range.clip(maxSpeed, 5, 90);
        if (controller1.stoppedChanging(Controller.Axis.RightStickX))
            targetRotation = normalizedYaw;
        double turnPower;
        if (rotationInput != 0) turnPower = rotationInput;
        else turnPower = Numbers.turnCorrectionSpeed(normalizedYaw, targetRotation);


        double verticalMovePower = moveXInput * Math.sin(-yawRad) + moveYInput * Math.cos(-yawRad);
        double horizontalMovePower = moveXInput * Math.cos(-yawRad) - moveYInput * Math.sin(-yawRad);
        if (RECORDING) {
            double currentTime = getRuntime() - idleRemoveTime;
            telemetry.addData("Remaining time:", 28-currentTime);
            if (29.5-currentTime <= 0) {
                DONE_RECORDING = true;
            }
            if (!idling) {
                saveRobotState(horizontalMovePower, verticalMovePower, rotationInput, rotationInput != 0 ? normalizedYaw : targetRotation,
                        chassis.collectionArmMotor.getCurrentPosition(), chassis.scoringArmMotor.getCurrentPosition(), chassis.bucket.getPosition(), chassis.claw.getPosition());
                chassis.move(horizontalMovePower, verticalMovePower, turnPower, maxSpeed);
            }
            else {
                telemetry.addLine("===== LAST SAVED SAVESTATE =====");
                telemetry.addLine("Currently idling!");
                telemetry.addLine("(input any input to continue recording)");
                chassis.move(0, 0, turnPower, maxSpeed);
            }
        }
        telemetry.update();
    }

    public void saveRobotState(double horzPow, double vertPow, double turnPow, double cYaw, int harmpos, int vertpos, double bucket, double claw) {
        SaveState latest = new SaveState(horzPow, vertPow, cYaw, getRuntime()-idleRemoveTime, maxSpeed, turnPow, harmpos, vertpos, bucket, claw, 0);
        states.add(latest);

        telemetry.addLine("===== LAST SAVED SAVESTATE =====");
        telemetry.addData("time", latest.t);
        telemetry.addData("mX", latest.mX);
        telemetry.addData("mY", latest.mY);
        telemetry.addData("yaw", latest.yaw);
    }

    private void serializeStates(String name) throws IOException {
        File directory = new File(PATH);
        if (!directory.exists()) {
            if (!directory.mkdirs()) {
                throw new IOException("Failed to create directory: " + PATH);
            }
        }
        FileOutputStream fileOutputStream = new FileOutputStream(PATH + name + ".robotStates");
        ObjectOutputStream objectOutputStream = new ObjectOutputStream(fileOutputStream);
        PathData data = new PathData(pathName, states);
        objectOutputStream.writeObject(data);
        objectOutputStream.flush();
        objectOutputStream.close();
        telemetry.addLine("Successfully saved to:\"" + PATH + "\"");
        telemetry.update();
    }
}
