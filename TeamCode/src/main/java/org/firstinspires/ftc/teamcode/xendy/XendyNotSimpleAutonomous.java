package org.firstinspires.ftc.teamcode.xendy;

import static org.firstinspires.ftc.teamcode.xendy.AutonomousSwitcherHelpers.loadStatesFromFile;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="XendyNotSimpleAutonomous")
public class XendyNotSimpleAutonomous extends OpMode {
    public String pathName = "";

    private XDriveChassis chassis;
    private YawPitchRollAngles orientation;
    private double yaw;
    private double yawRad;
    private double normalizedYaw;
    public static final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/XendysPessimisticAutonomous/";


    ArrayList<SaveState> states = new ArrayList<>();
    int index = 0;
    @Override
    public void init() {
        chassis = new XDriveChassis(this);
        AutonomousSwitcherHelpers.update();
        String binding = AutonomousSwitcherHelpers.selected;
        try {
            PathData data = loadStatesFromFile(binding);
            states = data.states;
            pathName = data.name;
            resetRuntime();
        } catch (IOException | ClassNotFoundException e) {
            e.printStackTrace();
            STOP = true;
        }
    }

    boolean STOP = false;
    @Override
    public void loop() {
        if (STOP) {
            return;
        }
        telemetry.addData("Running path", pathName);
        orientation = chassis.imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();
        yawRad = orientation.getYaw(AngleUnit.RADIANS);
        normalizedYaw = Numbers.normalizeAngle(yaw);

        double horizontalMovePower, verticalMovePower, turnPower, maxSpeed;
        SaveState currentState = states.get(index);
        while (currentState.t <= getRuntime()) {
            if (index >= states.size()) {
                break;
            }
            currentState = states.get(index);
            index += 1;
        }
        if (index >= states.size()) {
            currentState = null;
        }
        telemetry.addData("Running state index", index);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addLine("\nState data:");
        if (currentState != null) {
            telemetry.addData("time", currentState.t);
            telemetry.addData("mX", currentState.mX);
            telemetry.addData("mY", currentState.mY);
            telemetry.addData("yaw", currentState.yaw);
            telemetry.addData("maxSpeed", currentState.mS);
            horizontalMovePower = currentState.mX;
            verticalMovePower = currentState.mY;
            turnPower = currentState.turnPower - Numbers.turnCorrectionSpeed(normalizedYaw, currentState.yaw);
            maxSpeed = currentState.mS;
        }
        else {
            telemetry.addLine("Out of states.");
            horizontalMovePower = 0;
            verticalMovePower = 0;
            turnPower = 0;
            maxSpeed = 0;
            STOP = true;
        }
        chassis.move(horizontalMovePower, verticalMovePower, turnPower, maxSpeed);
        telemetry.update();
    }
}
