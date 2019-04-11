package org.firstinspires.ftc.teamcode.util.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import static java.lang.Math.abs;


/**
 * This class handles the gamepad's functionality
 */
public class GamepadManager {
    private final double ANALOG_THRESHOLD = 0.1;
    private final double SLOW_MULTIPLIER = 0.5;

    public Gamepad gamepad;

    public double left_axis_x;
    public double left_axis_y;

    public double right_axis_x;
    public double right_axis_y;

    public double left_trigger_value;
    public double right_trigger_value;

    public GamepadManager(Gamepad gamepad) { this.gamepad = gamepad; }

    /**
     * The left analog stick x-axis.
     *
     * @return value of left analog x-axis
     */
    public double getLeftX(boolean slowMode, boolean threshold) {
        if (threshold) {
            if (abs(gamepad.left_stick_x) < ANALOG_THRESHOLD) return 0;
            else {
                if (slowMode) return gamepad.left_stick_x * SLOW_MULTIPLIER;
                else return gamepad.left_stick_x;
            }
        } else {
            if (slowMode) return gamepad.left_stick_x * SLOW_MULTIPLIER;
            else return gamepad.left_stick_x;
        }
    }

    /**
     * The left analog stick y-axis.
     *
     * @return value of left analog y-axis (pushing stick up is negative)
     */
    public double getLeftY(boolean slowMode) {
        if (slowMode) { return gamepad.left_stick_y * SLOW_MULTIPLIER; }
        else return gamepad.left_stick_y;
    }

    /**
     * The right analog stick x-axis.
     *
     * @return value of right analog x-axis
     */
    public double getRightX(boolean slowMode) {
        if (slowMode) { return gamepad.right_stick_x * SLOW_MULTIPLIER; }
        else return gamepad.right_stick_x;
    }

    /**
     * The right analog stick y-axis.
     *
     * @return value of right analog y-axis (pushing stick up is negative)
     */
    public double getRightY(boolean slowMode) {
        if (slowMode) { return gamepad.right_stick_y * SLOW_MULTIPLIER; }
        else return gamepad.right_stick_y;
    }

    /**
     * The left analog trigger.
     *
     * @return value of left trigger
     */
    public double getLeftTrigger(boolean slowMode) {
        if (slowMode) { return gamepad.left_trigger * SLOW_MULTIPLIER; }
        else return gamepad.left_trigger;
    }

    /**
     * The right analog trigger.
     *
     * @return value of right trigger
     */
    public double getRightTrigger(boolean slowMode) {
        if (slowMode) { return gamepad.right_trigger * SLOW_MULTIPLIER; }
        else return gamepad.right_trigger;
    }
}