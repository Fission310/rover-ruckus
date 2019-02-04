package org.firstinspires.ftc.teamcode.opmode;

/**
 * Contains the constants for each step of the autonomous opmodes.
 */
public class Steps {
    public enum State {
        HANG_AND_SAMPLE,
        LAND,
        EXTEND_LIFT,
        STRAFE_OUT_LANDER,
        IMU_INIT,
        TURN_45,
        TURN_90,
        EXTEND_DRAWER_SLIDES,
        RETRACT_DRAWER_SLIDES,
        SCORE_IN_LANDER,
        FIND_GOLD_LOCATION,
        ALIGN_TO_GOLD,
        SAMPLE,
        DRIVE_TO_DEPOT,
        ALIGN_TO_WALL,
        MARKER,
        PARK,
        DEFAULT
    }
}