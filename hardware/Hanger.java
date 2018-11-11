package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Hanger is the class that is used to define all of the hardware for a robot's hanger.
 * Hanger must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the hanger.
 */
public class Hanger extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private DcMotor hanger;
    /**
     * Default constructor for Hanger.
     */
    public Hanger(){

    }
    /**
     * Overloaded constructor for hanger. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Hanger(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes hanger hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars

        // Retrieve motor from hardware map and assign to instance vars
        hanger = hwMap.dcMotor.get(RCConfig.Hanger);

        // Set braking behavior
        hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        hanger.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set initial power
        hanger.setPower(0);
    }

    /**
     * Sets power for hanger motor.
     */
    public void setHangerPower(double power) {
        hanger.setPower(power);
    }

    /**
     * Sets power for hanger motor based on encoder values.
     */
    public void hangerToPos(double power) {
        hanger.setPower(power);
    }

}