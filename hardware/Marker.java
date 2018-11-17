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
    private static final double SWEEPER_LEFT_POS = 0;
    private static final double SWEEPER_NEUTRAL_POS = 0.5;
    private static final double SWEEPER_RIGHT_POS = 1;

    /* Hardware members */
    private Servo arm;


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
        arm = hwMap.servo.get(RCConfig.MARKER);
        sweeperNeutral();
    }

    /**
     * Set the sweeper to the left position.
     */
    public void sweeperLeft() {
        arm.setPosition(SWEEPER_LEFT_POS);
    }

    /**
     * Set the sweeper to the left position.
     */
    public void sweeperRight() {
        arm.setPosition(SWEEPER_RIGHT_POS);
    }

    /**
     * Set the sweeper to the neutral position.
     */
    public void sweeperNeutral() {
        arm.setPosition(SWEEPER_NEUTRAL_POS);
    }
}