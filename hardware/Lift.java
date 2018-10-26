package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;


/**
 * Acquirer is the class that is used to define all of the hardware for a robot's acquirer.
 * Acquirer must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the acquirer.
 */
public class Lift extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DigitalChannel limitSwitch;
    /**
     * Default constructor for Acquirer.
     */
    public Lift(){

    }
    /**
     * Overloaded constructor for Lift. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Lift(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes lift hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars

        // Retrieve motor from hardware map and assign to instance vars
        leftLift = hwMap.dcMotor.get(RCConstants.LEFT_LIFT);
        rightLift = hwMap.dcMotor.get(RCConstants.RIGHT_LIFT);
        limitSwitch = hwMap.digitalChannel.get(RCConstants.LIMIT_SWITCH);

        // Set braking behavior
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        leftLift.setPower(0);
        rightLift.setPower(0);
    }

    /**
     * Sets power for lift motor.
     */
    public void setLiftPower(double power) {
        rightLift.setPower(power);
        leftLift.setPower(power);
    }

    /**
     * Sets power for lift motor based on encoder values.
     */
    public void liftToPos(double power) {
        rightLift.setPower(power);
        leftLift.setPower(power);
    }

}