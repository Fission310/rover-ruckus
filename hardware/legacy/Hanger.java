package org.firstinspires.ftc.teamcode.hardware.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RCConfig;


/**
 * Hanger is the class that is used to define all of the hardware for a robot's hanger.
 * Hanger must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the hanger.
 */
public class Hanger extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private DcMotor leftHanger;
    private DcMotor rightHanger;
    private DcMotor winch;
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
        // Retrieve servos from hardware map and assign to inst?ance vars

        // Retrieve motor from hardware map and assign to instance vars
        leftHanger = hwMap.dcMotor.get(RCConfig.LEFTHANGER);
        rightHanger = hwMap.dcMotor.get(RCConfig.RIGHTHANGER);
        winch = hwMap.dcMotor.get(RCConfig.WINCH);

        // Set braking behavior
        leftHanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        leftHanger.setDirection(DcMotorSimple.Direction.FORWARD);
        rightHanger.setDirection(DcMotorSimple.Direction.REVERSE);
        winch.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        leftHanger.setPower(0);
        rightHanger.setPower(0);
        winch.setPower(0);
    }

    /**
     * Sets power for hanger motor.
     */
    public void setHangerPower(double power) {
        leftHanger.setPower(power);
        rightHanger.setPower(power);
    }

    /**
     * Sets power for spool motor.
     */

    public void spool(double power) {
        winch.setPower(power);
    }

    /**
     * Sets power for hanger motor based on encoder values.
     */
    public void hangeToPos(double inches) {
        // Target position variables
        int newTarget = leftHanger.getCurrentPosition() + (int)(inches * Constants.TICKS_PER_INCH);

        // Determine new target position, and pass to motor controller
        leftHanger.setTargetPosition(newTarget);
        rightHanger.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        leftHanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightHanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftHanger.setPower(.2);
        rightHanger.setPower(.2);
    }
}