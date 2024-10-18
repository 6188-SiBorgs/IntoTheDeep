package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private final Gamepad currentGamepad = new Gamepad();
    private final Controller previousController;

    private float stickDeadzone = 0.01f;

    public Controller(Gamepad gamepad) {
        currentGamepad.copy(gamepad);
        previousController = new Controller();
    }

    /**
     * A private constructor for creating Controllers that are only to save the previous state of a controller
     */
    private Controller() {
        previousController = null;
    }

    public enum Button {
        // XBOX / Logitech
        A,
        B,
        X,
        Y,
        Guide,
        Back,
        Options,

        // Playstation
        Cross,
        Circle,
        Square,
        Triangle,
        Share,
        Start,
        PSButton,
        Touchpad,
        TouchpadFinger1,
        TouchpadFinger2,

        // Shared
        LeftBumper,
        RightBumper,
        LeftTrigger,
        RightTrigger,
        LeftStick,
        RightStick,
    }

    public enum Axis {
        LeftStickX,
        LeftStickY,
        RightStickX,
        RightStickY,
        LeftTrigger,
        RightTrigger,

        // Playstation only
        TouchpadFinger1X,
        TouchpadFinger1Y,
        TouchpadFinger2X,
        TouchpadFinger2Y,
    }

    public boolean getButton(Button button) {
        Gamepad g = currentGamepad;
        switch (button) {
            case A:
                return g.a;
            case B:
                return g.b;
            case X:
                return g.x;
            case Y:
                return g.y;
            case Guide:
                return g.guide;
            case Back:
                return g.back;
            case Options:
                return g.options;
            case Cross:
                return g.cross;
            case Circle:
                return g.circle;
            case Square:
                return g.square;
            case Triangle:
                return g.triangle;
            case Share:
                return g.share;
            case Start:
                return g.start;
            case PSButton:
                return g.ps;
            case Touchpad:
                return g.touchpad;
            case TouchpadFinger1:
                return g.touchpad_finger_1;
            case TouchpadFinger2:
                return g.touchpad_finger_2;
            case LeftBumper:
                return g.left_bumper;
            case RightBumper:
                return g.right_bumper;
            case LeftTrigger:
                return getAxis(Axis.LeftTrigger) > 0;
            case RightTrigger:
                return getAxis(Axis.RightTrigger) > 0;
            case LeftStick:
                return g.left_stick_button;
            case RightStick:
                return g.right_stick_button;
            default:
                throw new IllegalArgumentException();
        }
    }

    public float getAxis(Axis axis) {
        Gamepad g = currentGamepad;
        switch (axis) {
            case LeftStickX:
                return Numbers.deadzone(g.left_stick_x, stickDeadzone);
            case LeftStickY:
                return Numbers.deadzone(g.left_stick_y, stickDeadzone);
            case RightStickX:
                return Numbers.deadzone(g.right_stick_x, stickDeadzone);
            case RightStickY:
                return Numbers.deadzone(g.right_stick_y, stickDeadzone);
            case LeftTrigger:
                return g.left_trigger;
            case RightTrigger:
                return g.right_trigger;
            case TouchpadFinger1X:
                return g.touchpad_finger_1_x;
            case TouchpadFinger1Y:
                return g.touchpad_finger_1_y;
            case TouchpadFinger2X:
                return g.touchpad_finger_2_x;
            case TouchpadFinger2Y:
                return g.touchpad_finger_2_y;
            default:
                throw new IllegalArgumentException();
        }
    }

//    public boolean buttonWasPressed

    public Gamepad.Type getBrand() {
        return currentGamepad.type;
    }

    public void update(Gamepad gamepad) {
        if (previousController != null)
            previousController.update(currentGamepad);
        currentGamepad.copy(gamepad);
    }

    public float getStickDeadzone() {
        return stickDeadzone;
    }

    public void setStickDeadzone(float stickDeadzone) {
        this.stickDeadzone = stickDeadzone;
    }
}