package org.firstinspires.ftc.teamcode.xendy;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Numbers;
import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;
import org.firstinspires.ftc.teamcode.utils.controller.PowerCurve;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.PrintWriter;
import java.io.Serializable;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;
import java.util.stream.Collectors;

@Config
@TeleOp(name="XendyRecorder")
public class Recorder extends OpMode {
    public static String pathName = "";
    public String binding = "";
    private final double MAX_HESITATION_TIME = 0.3;

    private XDriveChassis chassis;
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


    ArrayList<SaveState> states = new ArrayList<>();
    int index = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        chassis = new XDriveChassis(this);
        pathName = "";
    }

    boolean DONE_RECORDING = false;
    boolean SENT_TO_TELEMETRY = false;
    boolean RECORDING = false;

    double idleStartTime = -1;
    double idleRemoveTime = 0;
    boolean idling = false;
    @Override
    public void loop() {
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
                    telemetry.addLine("Please choose a binding for this path.");
                    telemetry.addLine("(A/B/Y/X, Dpad)");
                    if (controller1.pressed(Controller.Button.A)) binding = "G1A";
                    else if (controller1.pressed(Controller.Button.B)) binding = "G1B";
                    else if (controller1.pressed(Controller.Button.X)) binding = "G1X";
                    else if (controller1.pressed(Controller.Button.Y)) binding = "G1Y";
                    else if (controller1.pressed(Controller.Button.DPadUp)) binding = "G1DpadUp";
                    else if (controller1.pressed(Controller.Button.DPadDown)) binding = "G1DpadDown";
                    else if (controller1.pressed(Controller.Button.DPadRight)) binding = "G1DpadRight";
                    else if (controller1.pressed(Controller.Button.DPadLeft)) binding = "G1DpadLeft";
                }
                else {
                    try {
                        serializeStates(binding);
                    } catch (IOException e) {
                        telemetry.addLine(formatIOException(e));
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

        float moveXInput = -controller1.axis(Controller.Axis.LeftStickX, PowerCurve.Quadratic);
        float moveYInput = controller1.axis(Controller.Axis.LeftStickY, PowerCurve.Quadratic);
        float rotationInput = controller1.axis(Controller.Axis.RightStickX, PowerCurve.Cubic);



        if ((moveXInput != 0 || moveYInput != 0 || rotationInput != 0) && !RECORDING) {
            RECORDING = true;
            resetRuntime();
            chassis.imu.resetYaw();
            yaw = 0;
            yawRad = 0;
            normalizedYaw = 0;
        }

        if (RECORDING && moveXInput == 0 && moveYInput == 0 && rotationInput == 0) {
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
        else turnPower = -Numbers.turnCorrectionSpeed(normalizedYaw, targetRotation);


        double verticalMovePower = moveXInput * Math.sin(-yawRad) + moveYInput * Math.cos(-yawRad);
        double horizontalMovePower = moveXInput * Math.cos(-yawRad) - moveYInput * Math.sin(-yawRad);
        if (RECORDING) {
            double currentTime = getRuntime() - idleRemoveTime;
            telemetry.addData("Remaining time:", 28-currentTime);
            if (29.5-currentTime <= 0) {
                DONE_RECORDING = true;
            }
            if (!idling) {
                saveRobotState(horizontalMovePower, verticalMovePower, rotationInput, rotationInput != 0 ? normalizedYaw : targetRotation, 0, 0, 0, 0);
                chassis.move(horizontalMovePower, verticalMovePower, turnPower, maxSpeed);
            }
            else {
                telemetry.addLine("===== LAST SAVED SAVESTATE =====");
                telemetry.addLine("Currently idling!");
                telemetry.addData("Max Hesitation Time: ", MAX_HESITATION_TIME);
                chassis.move(0, 0, turnPower, maxSpeed);
            }
        }
        telemetry.update();
    }

    public void saveRobotState(double horzPow, double vertPow, double turnPow, double cYaw, double harmpos, double vertpos, double bucket, double claw) {
        SaveState latest = new SaveState(horzPow, vertPow, cYaw, getRuntime()-idleRemoveTime, maxSpeed, turnPow, harmpos, vertpos, bucket, claw);
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
        telemetry.addLine("Successfully saved to:\"" + PATH+"\"");
        telemetry.update();
    }

    public static String formatIOException(IOException e) {
        StringWriter stringWriter = new StringWriter();
        PrintWriter printWriter = new PrintWriter(stringWriter);

        // Write stack trace to a String
        e.printStackTrace(printWriter);
        String stackTrace = stringWriter.toString();

        // Extract cause
        String cause = (e.getCause() != null) ? e.getCause().toString() : "No cause";

        // Extract location (where it happened)
        StackTraceElement[] stackTraceElements = e.getStackTrace();
        String location = stackTraceElements.length > 0
                ? stackTraceElements[0].toString()
                : "No stack trace available";

        // Format output
        return String.format(
                "Exception: %s%nCause: %s%nLocation: %s%nStack Trace:%n%s",
                e.toString(),
                cause,
                location,
                stackTrace
        );
    }

}
