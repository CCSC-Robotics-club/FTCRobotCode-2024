package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.Utils.MathUtils.Vector2D;

import java.util.HashMap;
import java.util.Map;

public class DriverGamePad {
    public final Gamepad rawGamePad;

    public final Map<RobotConfig.XboxControllerKey, Boolean> xBoxControllerKeyOnHoldMap, xBoxControllerKeyOnHoldMapPrevious, xBoxControllerKeyOnPressedMap, xBoxControllerKeyOnReleaseMap;
    public DriverGamePad(Gamepad rawGamePad) {
        this.rawGamePad = rawGamePad;
        this.xBoxControllerKeyOnHoldMap = new HashMap<>(1);
        this.xBoxControllerKeyOnHoldMapPrevious = new HashMap<>(1);
        this.xBoxControllerKeyOnPressedMap = new HashMap<>(1);
        this.xBoxControllerKeyOnReleaseMap = new HashMap<>(1);
    }

    private void updateKeyHoldStatus() {
        if (rawGamePad.a)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.A, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.A, false);

        if (rawGamePad.b)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.B, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.B, false);

        if (rawGamePad.x)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.X, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.X, false);

        if (rawGamePad.y)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.Y, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.Y, false);

        if (rawGamePad.dpad_up)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.DPAD_UP, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.DPAD_UP, false);

        if (rawGamePad.dpad_down)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.DPAD_DOWN, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.DPAD_DOWN, false);

        if (rawGamePad.dpad_left)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.DPAD_LEFT, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.DPAD_LEFT, false);

        if (rawGamePad.dpad_right)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.DPAD_RIGHT, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.DPAD_RIGHT, false);

        if (rawGamePad.left_stick_button)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.LEFT_STICK_BUTTON, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.LEFT_STICK_BUTTON, false);

        if (rawGamePad.right_stick_button)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.RIGHT_STICK_BUTTON, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.RIGHT_STICK_BUTTON, false);

        if (rawGamePad.left_bumper)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.LEFT_BUMPER, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.LEFT_BUMPER, false);

        if (rawGamePad.right_bumper)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.RIGHT_BUMPER, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.RIGHT_BUMPER, false);

        if (rawGamePad.left_trigger > RobotConfig.ControlConfigs.triggerThreshold)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.LEFT_TRIGGER, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.LEFT_TRIGGER, false);

        if (rawGamePad.right_trigger > RobotConfig.ControlConfigs.triggerThreshold)
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.RIGHT_TRIGGER, true);
        else
            xBoxControllerKeyOnHoldMap.put(RobotConfig.XboxControllerKey.RIGHT_TRIGGER, false);
    }

    private void updateKeyOnPressed() {
        for (RobotConfig.XboxControllerKey key: RobotConfig.XboxControllerKey.values()) {
            if (xBoxControllerKeyOnHoldMapPrevious.containsKey(key))
                this.xBoxControllerKeyOnPressedMap.put(key,
                        keyOnHold(key) &&
                        Boolean.FALSE.equals(xBoxControllerKeyOnHoldMapPrevious.get(key))); // a key is on pressed if it is pressed now and not pressed before
            if (xBoxControllerKeyOnHoldMapPrevious.containsKey(key))
                this.xBoxControllerKeyOnReleaseMap.put(key, !keyOnHold(key) &&
                        Boolean.TRUE.equals(xBoxControllerKeyOnHoldMapPrevious.get(key))); // a key is on pressed if it is pressed now and not pressed before

            xBoxControllerKeyOnHoldMapPrevious.put(key, xBoxControllerKeyOnHoldMap.get(key)); // update previous one
        }
    }

    public void update() {
        updateKeyHoldStatus();
        updateKeyOnPressed();
    }

    /** whether the key is held at the moment */
    public boolean keyOnHold(RobotConfig.XboxControllerKey key) {
        return Boolean.TRUE.equals(
                this.xBoxControllerKeyOnHoldMap.get(key));
    }

    public boolean keyOnPressed(RobotConfig.XboxControllerKey key) {
        return Boolean.TRUE.equals(
                this.xBoxControllerKeyOnPressedMap.get(key));
    }

    public boolean keyOnReleased(RobotConfig.XboxControllerKey key) {
        return Boolean.TRUE.equals(
                this.xBoxControllerKeyOnReleaseMap.get(key)
        );
    }

    public double getRotationStickValue() {
        final double rawStickValue;
        switch (RobotConfig.ControlConfigs.translationalControllerStick) {
            case LEFT_HAND: {
                rawStickValue = rawGamePad.right_stick_x; // the other axis
                break;
            } case RIGHT_HAND: {
                rawStickValue = rawGamePad.left_stick_x;
                break;
            } default: throw new IllegalArgumentException("unknown stick " + RobotConfig.ControlConfigs.translationalControllerStick);
        }
        return applyExp(rawStickValue, RobotConfig.ControlConfigs.pilotController_rotationalStickExp) * RobotConfig.ControlConfigs.pilotController_rotationSensitivity;
    }

    public Vector2D getTranslationStickVector() {
        final double rawStickX, rawStickY;
        switch (RobotConfig.ControlConfigs.translationalControllerStick) {
            case RIGHT_HAND: {
                rawStickX = rawGamePad.right_stick_x;
                rawStickY = rawGamePad.right_stick_y;
                break;
            } case LEFT_HAND: {
                rawStickX = rawGamePad.left_stick_x;
                rawStickY = rawGamePad.left_stick_y;
                break;
            } default: throw new IllegalArgumentException("unknown stick " + RobotConfig.ControlConfigs.translationalControllerStick);
        }
        final double stickX = applyControllerDeadBand(rawStickX, rawStickY), stickY = applyControllerDeadBand(rawStickY, rawStickX),
                translationX = applyExp(stickX, RobotConfig.ControlConfigs.pilotController_translationStickXExp) * RobotConfig.ControlConfigs.pilotController_translationStickXSensitivity,
                translationY = applyExp(stickY, RobotConfig.ControlConfigs.pilotController_translationStickYExp) * RobotConfig.ControlConfigs.pilotController_translationStickYSensitivity;
        return new Vector2D(new double[] {translationX, translationY});
    }
    private static double applyExp(double value, double exp) {
        return Math.copySign(
                Math.pow(Math.abs(value), exp),
                value
        );
    }

    private double applyControllerDeadBand(double axis, double otherAxis) {
        final double deadBand =  RobotConfig.ControlConfigs.pilotController_deadBand
                + (Math.abs(otherAxis)) * (RobotConfig.ControlConfigs.pilotController_fullStickDeadBand - RobotConfig.ControlConfigs.pilotController_deadBand);

        if (Math.abs(axis) < deadBand)
            axis = 0;
        return axis;
    }
}
