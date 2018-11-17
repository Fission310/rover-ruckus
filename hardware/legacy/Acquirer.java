package org.firstinspires.ftc.teamcode.hardware.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RCConfig;


/**
 * Acquirer is the class that is used to define all of the hardware for a robot's acquirer.
 * Acquirer must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the acquirer.
 */
public class Acquirer extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private CRServo intakeMotor;
    private Servo acquirerFloor;
    /**
     * Default constructor for Acquirer.
     */
    public Acquirer(){

    }
    /**
     * Overloaded constructor for Acquirer. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Acquirer(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes acquirer hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars

        // Retrieve motor from hardware map and assign to instance vars
        intakeMotor = hwMap.crservo.get(RCConfig.ACQUIRER_INTAKE);

        // Set polarity
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        intakeMotor.setPower(0);
    }

    /**
     * Sets power for intake motor.
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

}