package org.firstinspires.ftc.teamcode.xendy;

import android.os.Environment;

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

@TeleOp(name="XendyRecorder")
public class Recorder extends OpMode {
    private final String LOAD_FROM_PATH = "";

    private XDriveChassis chassis;
    private double maxSpeed = 50;

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
        chassis = new XDriveChassis(this);
    }


    public void loadStatesFromFile(String name) throws IOException, ClassNotFoundException {
        FileInputStream fileInputStream = new FileInputStream(PATH + name + ".robotStates");
        ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
        states = (ArrayList<SaveState>) objectInputStream.readObject();
        objectInputStream.close();
    }

    boolean DONE_RECORDING = false;
    boolean SENT_TO_TELEMETRY = false;
    boolean RECORDING = false;

    @Override
    public void loop() {
        controller1.update(gamepad1);
        controller2.update(gamepad2);
        if (controller1.pressed(Controller.Button.Start) && LOAD_FROM_PATH.isEmpty()) {
            DONE_RECORDING = true;
        }
        if (DONE_RECORDING) {
            if (!SENT_TO_TELEMETRY) {
                try {
                    serializeStates("recorded");
                } catch (IOException e) {
                    telemetry.addLine(formatIOException(e));
                    telemetry.update();
                }
                SENT_TO_TELEMETRY = true;
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


        double verticalMovePower = moveXInput * Math.sin(yawRad) + moveYInput * Math.cos(yawRad);
        double horizontalMovePower = moveXInput * Math.cos(yawRad) - moveYInput * Math.sin(yawRad);
        if (RECORDING) {
            saveRobotState(horizontalMovePower, verticalMovePower, rotationInput, rotationInput != 0 ? normalizedYaw : targetRotation);
            chassis.move(horizontalMovePower, verticalMovePower, turnPower, maxSpeed);
        }
        telemetry.update();
    }

    public void saveRobotState(double horzPow, double vertPow, double turnPow, double cYaw) {
        SaveState latest = new SaveState(horzPow, vertPow, cYaw, getRuntime(), maxSpeed, turnPow);
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
        objectOutputStream.writeObject(states);
        objectOutputStream.flush();
        objectOutputStream.close();
        telemetry.addLine("Successfully saved to:\"" + PATH+"\"");
        telemetry.update();
    }

    String SERIALIZED_FINAL_STATES = "";
    private void createSerializedStates() {
        SERIALIZED_FINAL_STATES =
                states.stream().map(s->String.format(
                        "%s:%s:%s:%s",
                        s.t, s.mX, s.mY, s.yaw
                )).collect(Collectors.joining("|"));
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
