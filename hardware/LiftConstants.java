package org.firstinspires.ftc.teamcode.hardware;

/**
 * Contains the all general constants for robot's lift.
 */

public class LiftConstants {
    /* CONSTANTS */
    private static final double PPR = 7.0;
    private static final double CPR = PPR * 4.0; // 28
    private static final double GEAR_RATIO_3_7 = 20;

    /**
     * Ticks per revolution for a NeverRest motors.
     */
    private static final double TICKS_PER_MOTOR = CPR * GEAR_RATIO_3_7;

    /**
     * Motor to leadscrew gear ratio (< 1.0 if geared up).
     */
    private static final double MOTOR_T0_LEADSCREW = 1.0;

    /**
     * Ticks per Leadscrew
     */
    public static final double TICKS_PER_LEADSCREW = TICKS_PER_MOTOR * MOTOR_T0_LEADSCREW;

    /**
     * Liner Distance per rotation of leadscrew  in inches.
     */
    public static final double LINEAR_TRAVEL_PER_ROTATION_INCH = (1.0 / 25.4) * 8.0; // MM to INCH

    /**
     * Ticks per revolution of linear lift
     */
    public static final double TICKS_PER_LINEAR_LIFT_INCH = TICKS_PER_LEADSCREW / LINEAR_TRAVEL_PER_ROTATION_INCH;

    /**
     * Total length of extension in inches
     */
    public static final double STROKE_LENGTH = 7.4; // INCH

    /**
     * Total length of extension in ticks
     */
    public static final double TOTAL_STROKE_TICKS = TICKS_PER_LINEAR_LIFT_INCH * STROKE_LENGTH;

    /**
     * Retracted position in encoder ticks
     */
    public static final int RETRACTED_POSITION = (int)(LINEAR_TRAVEL_PER_ROTATION_INCH * .25);

    /**
     * Extended position in encoder ticks
     */
    public static final int EXTENDED_POSITION = (int)(TOTAL_STROKE_TICKS - LINEAR_TRAVEL_PER_ROTATION_INCH * 3.6);
}
