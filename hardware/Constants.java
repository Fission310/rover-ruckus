package org.firstinspires.ftc.teamcode.hardware;

/**
 * Contains the all general constants for robot.
 */

public class Constants {
    /* CONSTANTS */
    /**
     * Ticks per revolution for a NeverRest 40.
     */
    public static final double     COUNTS_PER_MOTOR_REV    = 1120;
    /**
     * Drivetrain gear ratio (< 1.0 if geared up).
     */
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    /**
     * Diameter of wheel in inches.
     */
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    /**
     * Calculated ticks per inch.
     */
    public static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    /**
     * Drive speed when using encoders.
     */
    public static final double     DRIVE_SPEED             = 0.4;
    /**
     * Turn speed when using encoders.
     */
    public static final double     TURN_SPEED              = 0.3;

    // Constant adjusting value for encoder driving
    public static final double     PCONSTANT               = 0.1;
}
