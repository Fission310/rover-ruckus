package org.firstinspires.ftc.teamcode.hardware;

/**
 * Contains the all general constants for robot.
 */

public class Constants {
    /* CONSTANTS */
    public static final double NEVEREST_GEAR_RATIO  = 60;
    public static final double MODERN_ROBOTICS_GEAR_RATIO = 25.83;
    public static final double PPR = 7.0;
    public static final double CPR = PPR * 4;
    public static final double CPRRACK = PPR * 2;
    /**
     * Ticks per revolution for a NeverRest 40 and GoBilda.
     */
    public static final double TICKS_PER_MOTOR_REV = CPR * NEVEREST_GEAR_RATIO; // 1120
    public static final double TICKS_PER_MOTOR_MR = CPRRACK * MODERN_ROBOTICS_GEAR_RATIO; // 723.24
    /**
     * Drivetrain gear ratio (< 1.0 if geared up).
     */
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    /**
     * Diameter of wheel in inches.
     */
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    /**
     * Calculated ticks per inch.
     */
    public static final double INCHES_PER_TICK = ((WHEEL_DIAMETER_INCHES * Math.PI) / TICKS_PER_MOTOR_REV);
    public static final double INCHES_PER_TICK_MR = ((WHEEL_DIAMETER_INCHES * Math.PI) / TICKS_PER_MOTOR_MR);
    /**
     * Calculated inch per tick.
     */
    public static final double TICKS_PER_INCH = 1 / INCHES_PER_TICK;
    public static final double TICKS_PER_INCH_MR = 1 / INCHES_PER_TICK_MR;
    /**
     * Drive speed when using encoders.
     */
    public static final double DRIVE_SPEED = 0.4;
    /**
     * Turn speed when using encoders.
     */
    public static final double TURN_SPEED = 0.3;

    public static final double PCONSTANT = 0.1;
    /**
     * Ticks per revolution for a VEX EDR 393.
     */
    public static final double  COUNTS_PER_SERVO_VEX = 627.2;
}
