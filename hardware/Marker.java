package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Marker is the class that is used to define all of the hardware for a robot's marker arn.
 * Marker must be instantiated, then initialized using <code>init()</code> before being used.
 */
public class Marker extends Mechanism {

    /* CONSTANTS */
    private static final double MARKER_LEFT_POS = 0;
    private static final double MARKER_NEUTRAL_POS = 0.5;
    private static final double MARKER_RIGHT_POS = 1;

    /* Hardware members */
    private Servo marker;

    /**
     * Default constructor for Arm.
     */
    public Marker(){ }
    /**
     * Overloaded constructor for Arm. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Marker(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes arm hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve arm from hardware map and set to initial position
        marker = hwMap.servo.get(RCConfig.MARKER);
    }

    /**
     * Set the marker to the left position.
     */
    public void markerLeft() {
        marker.setPosition(MARKER_LEFT_POS);
    }

    /**
     * Set the marker to the left position.
     */
    public void markerRight() {
        marker.setPosition(MARKER_RIGHT_POS);
    }

    /**
     * Set the marker to the neutral position.
     */
    public void markerNeutral() {
        marker.setPosition(MARKER_NEUTRAL_POS);
    }
}