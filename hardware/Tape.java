package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Acquirer is the class that is used to define all of the hardware for a robot's acquirer.
 * Acquirer must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the acquirer.
 */
public class Tape extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private DcMotor tape;

    /**
     * Default constructor for Acquirer.
     */
    public Tape(){

    }
    /**
     * Overloaded constructor for Lift. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Tape(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes lift hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars

        // Retrieve motor from hardware map and assign to instance vars
        tape = hwMap.dcMotor.get(RCConstants.LIFT);

        // Set braking behavior
        tape.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        tape.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set initial power
        tape.setPower(0);
    }

    /**
     * Sets power for lift motor.
     */
    public void setTapePower(double power) {
        tape.setPower(power);
    }

    /**
     * Sets power for lift motor based on encoder values.
     */
    public void liftTapePos(double power) {
        tape.setPower(power);
    }

}