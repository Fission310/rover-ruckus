package org.firstinspires.ftc.teamcode.hardware.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RCConfig;


/**
 * Lift is the class that is used to define all of the hardware for a robot's loft.
 * Lift must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the lift.
 */
public class Lift extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private DcMotor leftLift;
    private DcMotor rightLift;
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
        leftLift = hwMap.dcMotor.get(RCConfig.LEFT_LIFT);
        rightLift = hwMap.dcMotor.get(RCConfig.RIGHT_LIFT);

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