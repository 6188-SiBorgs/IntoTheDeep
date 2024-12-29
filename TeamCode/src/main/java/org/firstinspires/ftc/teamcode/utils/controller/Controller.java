package org.firstinspires.ftc.teamcode.utils.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.Numbers;

import java.util.HashSet;

public class Controller {
    protected final Gamepad currentGamepad = new Gamepad();
    protected final HashSet<Button> pressedButtons = new HashSet<>();
    private Button mostRecentlyPressedButton = null;

    private float stickDeadzone = 0.05f;

    public Controller() {}

    public Controller(Gamepad gamepad) {
        currentGamepad.copy(gamepad);
    }

    public enum Button {
        // Dpads
        DPadLeft,
        DPadRight,
        DPadUp,
        DPadDown,

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

    public boolean button(Button button) {
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
                return axis(Axis.LeftTrigger) > 0;
            case RightTrigger:
                return axis(Axis.RightTrigger) > 0;
            case LeftStick:
                return g.left_stick_button;
            case RightStick:
                return g.right_stick_button;
            case DPadDown:
                return g.dpad_down;
            case DPadUp:
                return g.dpad_up;
            case DPadLeft:
                return g.dpad_left;
            case DPadRight:
                return g.dpad_right;
            default:
                throw new IllegalArgumentException();
        }
    }

    public float axis(Axis axis) {
        return axis(axis, PowerCurve.Linear);
    }

    public float axis(Axis axis, PowerCurve curve) {
        Gamepad g = currentGamepad;
        float value = 0;
        switch (axis) {
            case LeftStickX:
                value =  Numbers.deadzone(g.left_stick_x, stickDeadzone);
                break;
            case LeftStickY:
                value =  Numbers.deadzone(-g.left_stick_y, stickDeadzone);
                break;
            case RightStickX:
                value =  Numbers.deadzone(g.right_stick_x, stickDeadzone);
                break;
            case RightStickY:
                value =  Numbers.deadzone(-g.right_stick_y, stickDeadzone);
                break;
            case LeftTrigger:
                value =  g.left_trigger;
                break;
            case RightTrigger:
                value =  g.right_trigger;
                break;
            case TouchpadFinger1X:
                value =  g.touchpad_finger_1_x;
                break;
            case TouchpadFinger1Y:
                value =  g.touchpad_finger_1_y;
                break;
            case TouchpadFinger2X:
                value =  g.touchpad_finger_2_x;
                break;
            case TouchpadFinger2Y:
                value =  g.touchpad_finger_2_y;
                break;
        }

        return curve.apply(value);
    }

    public Gamepad.Type getBrand() {
        return currentGamepad.type;
    }

    public void update(Gamepad gamepad) {
        currentGamepad.copy(gamepad);
        for (Button button : Button.values()) {
            if (button(button)) {
                pressedButtons.add(button);
                mostRecentlyPressedButton = button;
            } else {
                pressedButtons.remove(button);
            }
        }
    }

    public float getStickDeadzone() {
        return stickDeadzone;
    }

    public void setStickDeadzone(float stickDeadzone) {
        this.stickDeadzone = stickDeadzone;
    }

    public Button getMostRecentlyPressedButton() {
        return mostRecentlyPressedButton;
    }
}
