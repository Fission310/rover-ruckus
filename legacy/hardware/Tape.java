package org.firstinspires.ftc.teamcode.legacy.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RCConfig;


/**
 * Tape is the class that is used to define all of the hardware for a robot's hanging mechanism.
 * Tape must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the tape.
 */
public class Tape extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private CRServo leftTape;
    private CRServo rightTape;
    private DcMotor winch;
    private DigitalChannel limitSwitch;

    /**
     * Default constructor for Tape.
     */
    public Tape(){

    }
    /**
     * Overloaded constructor for Tape. Sets the OpMode context.
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
        leftTape = hwMap.crservo.get(RCConfig.LEFT_TAPE);
        rightTape = hwMap.crservo.get(RCConfig.RIGHT_TAPE);
        winch = hwMap.dcMotor.get(RCConfig.WINCH);
        limitSwitch = hwMap.digitalChannel.get(RCConfig.LIMIT_SWITCH);

        // Set polarity
        leftTape.setDirection(CRServo.Direction.FORWARD);
        rightTape.setDirection(CRServo.Direction.REVERSE);
        winch.setDirection(DcMotor.Direction.FORWARD);


        // Set initial power
        leftTape.setPower(0);
        rightTape.setPower(0);
        winch.setPower(0);
    }

    /**
     * Sets power for tape motor.
     */
    public void setTapePower(double power) {
        leftTape.setPower(power);
        rightTape.setPower(power);
        winch.setPower(-power);
    }

    /**
     * Sets power for tape motor based on encoder values.
     */
    public void liftTapePos(double power) {
        leftTape.setPower(power);
        rightTape.setPower(power);
        winch.setPower(-power);
    }

}