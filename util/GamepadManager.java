package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This class handles the gamepad's functionality
 */
public class GamepadManager {
    public Gamepad gamepad;

    public static double ANALOG_THRESHOLD = 0.0;
    public static double SLOW_MULTIPLIER = 0.2;
    public static double FAST_MULTIPLIER = 2.0;

    /* Holds gamepad joystick's values */
    public double yInput = (double)-left_stick_y(), xInput = (double)right_stick_x(), slideInput = (double)left_stick_x();

    /* Applies slow or fast mode */
    public double slowYInput = yInput * SLOW_MULTIPLIER, slowXInput = xInput * SLOW_MULTIPLIER, slowSlide = slideInput * SLOW_MULTIPLIER;
    public double fastYInput = yInput * FAST_MULTIPLIER, fastXInput = xInput * FAST_MULTIPLIER, fastSlide = slideInput * FAST_MULTIPLIER;

    public GamepadManager(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void init(double threshold, double slow, double fast) {
        ANALOG_THRESHOLD = threshold;
        SLOW_MULTIPLIER = slow;
        FAST_MULTIPLIER = fast;
    }

    public double left_stick_x() { return gamepad.left_stick_x; }
    public double left_stick_y() { return gamepad.left_stick_y; }
    public double right_stick_x() { return gamepad.right_stick_x; }
    public double right_stick_y() { return gamepad.right_stick_y; }
    public double right_trigger() { return gamepad.right_trigger; }
    public double left_trigger() { return gamepad.left_trigger; }
    public boolean right_bumper() { return gamepad.right_bumper; }
    public boolean left_bumper() { return gamepad.left_bumper; }
}
