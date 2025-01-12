package org.firstinspires.ftc.teamcode.xendy;

import static org.firstinspires.ftc.teamcode.xendy.AutoUtils.loadStatesFromFile;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Numbers;

import java.io.IOException;
import java.util.ArrayList;

@Autonomous(name="XendyNotSimpleAutonomous")
public class XendyNotSimpleAutonomous extends OpMode {
    public String pathName = "";

    private DriveChassisX chassis;
    private YawPitchRollAngles orientation;
    private double yaw;
    private double yawRad;
    private double normalizedYaw;
    public static final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/XendysPessimisticAutonomous/";

    ArrayList<SaveState> states = new ArrayList<>();
    int index = 0;

    @Override
    public void init() {
        telemetry.addLine("DO NOT START THE PROGRAM");
        telemetry.addLine("Syncing AutoUtils...");
        telemetry.update();
        chassis = new DriveChassisX(this);
        chassis.isAuto();
        chassis.endPivotMotor.setVelocity(150);
        chassis.scoringArmMotor.setVelocity(910);
        chassis.collectionArmMotor.setVelocity(500);
        AutoUtils.update();
        String binding = AutoUtils.selected;
        telemetry.addLine("DO NOT START THE PROGRAM");
        telemetry.addLine("Reading states...");
        telemetry.update();
        try {
            PathData data = loadStatesFromFile(binding);
            states = data.states;
            pathName = data.name;
            telemetry.update();
        } catch (IOException e) {
            telemetry.addLine("===== CRITICAL ERROR =====");
            telemetry.addLine("FAILED TO READ PATH DATA AND NAME.");
            telemetry.addLine(AutoUtils.formatIOException(e));
            telemetry.update();
            e.printStackTrace();
            STOP = true;
        }
        catch (ClassNotFoundException ignore) {
            telemetry.addLine("===== CRITICAL ERROR =====");
            telemetry.addLine("Something has gone HORRENDOUSLY wrong.");
            telemetry.addLine("If you see this, then uh... ur screwed !");
            STOP = true;
        }
        telemetry.addLine("Ready to start!");
        telemetry.addLine(String.format("We are about to run \"%s\" binded to \"\".", pathName, binding));
        telemetry.addLine("If you do not want to run this auto, please run the \"SetPrimaryAutonomous\" teleop.");
        telemetry.addLine(String.format("Found %s states.", states.size()));
        telemetry.update();
    }

    boolean STOP = false;
    boolean first = true;
    @Override
    public void loop() {
        if (first) {
            chassis.ascention.setTargetPosition(-2380);
            chassis.ascention.setPower(1);
            chassis.ascention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            resetRuntime();
            first = false;
        }
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
            telemetry.addData("bucketPos", currentState.bucketPosition);
            telemetry.addData("clawPos", currentState.clawPosition);
            telemetry.addData("horzArmPos", currentState.horizArmPosition);
            telemetry.addData("vertArmPos", currentState.vertArmPosition);
            telemetry.addData("pivotPos", currentState.pivotPosition);

            horizontalMovePower = currentState.mX;
            verticalMovePower = currentState.mY;
            turnPower = currentState.turnPower + Numbers.turnCorrectionSpeed(normalizedYaw, currentState.yaw);
            maxSpeed = currentState.mS;
            chassis.bucket.setPosition(currentState.bucketPosition);
            chassis.claw.setPosition(currentState.clawPosition);
            chassis.scoringArmMotor.setTargetPosition(currentState.vertArmPosition);
            chassis.collectionArmMotor.setTargetPosition(currentState.horizArmPosition);
            chassis.endPivotMotor.setTargetPosition(currentState.pivotPosition);
//          // chassis.scoringArmMotor.setVelocity((chassis.scoringArmMotor.getTargetPosition() - chassis.scoringArmMotor.getCurrentPosition())/(getRuntime()-));
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
