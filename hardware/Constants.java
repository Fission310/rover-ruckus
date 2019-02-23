package org.firstinspires.ftc.teamcode.hardware;

/**
 * Contains the all general constants for robot.
 */

public class Constants {
    /* CONSTANTS */
    private static final double GEAR_RATIO_60 = 60.0;
    private static final double GEAR_RATIO_40 = 40.0;
    private static final double GEAR_RATIO_30 = 30.0;
    private static final double GEAR_RATIO_20 = 20.0;
    private static final double GEAR_RATIO_MR_53 = 53.475;
    private static final double PPR = 7.0;
    private static final double CPR = PPR * 4.0;
    /**
     * Ticks per revolution for a NeverRest and GoBilda motors.
     */
    private static final double TICKS_PER_MOTOR_60 = CPR * GEAR_RATIO_60; // 1680
    private static final double TICKS_PER_MOTOR_40 = CPR * GEAR_RATIO_40; // 1120
    private static final double TICKS_PER_MOTOR_30 = CPR * GEAR_RATIO_30; // 840
    private static final double TICKS_PER_MOTOR_20 = CPR * GEAR_RATIO_20; // 560
    private static final double TICKS_PER_MOTOR_53 = CPR * GEAR_RATIO_MR_53; // 723.24
    /**
     * Drivetrain gear ratio (< 1.0 if geared up).
     */
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double DRIVETRAIN_GEAR_REDUCTION = 1.0 / 3.0;
    private static final double ROTATION_GEAR_REDUCTION = 1.0;
    /**
     * Diameter of wheel in inches.
     */
    public static final double WHEEL_DIAMETER_INCHES_4 = 4.0;
    private static final double WHEEL_DIAMETER_INCHES_2 = 2.0;
    private static final double PINION_INCHES = 1.0 / 2.0;
    /**
     * Calculated ticks per inch.
     */
    public static final double INCHES_PER_TICK_60 = ((WHEEL_DIAMETER_INCHES_2 * Math.PI) / (TICKS_PER_MOTOR_60 * DRIVE_GEAR_REDUCTION));
    public static final double INCHES_PER_TICK_40 = ((WHEEL_DIAMETER_INCHES_4 * Math.PI) / (TICKS_PER_MOTOR_40 * DRIVETRAIN_GEAR_REDUCTION));
    public static final double INCHES_PER_TICK_30 = ((WHEEL_DIAMETER_INCHES_4 * Math.PI) / (TICKS_PER_MOTOR_30 * DRIVE_GEAR_REDUCTION));
    public static final double INCHES_PER_TICK_53 = ((WHEEL_DIAMETER_INCHES_2 * Math.PI) / (TICKS_PER_MOTOR_53 * ROTATION_GEAR_REDUCTION));
    public static final double INCHES_PER_TICK_RACK_PINION = ((PINION_INCHES * Math.PI) / (TICKS_PER_MOTOR_60 * DRIVE_GEAR_REDUCTION));
    /**
     * Calculated inch per tick.
     */
    public static final double TICKS_PER_INCH_60 = 1.0 / INCHES_PER_TICK_60;
    public static final double TICKS_PER_INCH_40 = 1.0 / INCHES_PER_TICK_40;
    public static final double TICKS_PER_INCH_30 = 1.0 / INCHES_PER_TICK_30;
    public static final double TICKS_PER_INCH_53 = 1.0 / INCHES_PER_TICK_53;
    public static final double TICKS_PER_INCH_RACK_PINION = 1.0 / INCHES_PER_TICK_RACK_PINION;
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
    public static final double  TICKS_PER_SERVO_VEX = 627.2;
}
