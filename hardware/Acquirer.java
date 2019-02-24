package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Acquirer is the class that is used to define all of the hardware for a robot's acquirer.
 * Acquirer must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the acquirer.
 */
public class Acquirer extends Mechanism {

    /* CONSTANTS */
    private static final double SERVO_INIT_POS = 0;
    private static final double SERVO_CENTER_POS = 1;

    /* Hardware members */
    private DcMotor intakeMotor;
    private Servo acquirerRotation;

    /**
     * Default constructor for Acquirer.
     */
    public Acquirer(){ }

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
        intakeMotor = hwMap.dcMotor.get(RCConfig.INTAKE_MOTOR);
//        acquirerRotation = hwMap.servo.get(RCConfig.ACQUIRER_ROTATION);

        // Set polarity
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set initial power
        intakeMotor.setPower(0);
    }

    /**
     * Sets power for intake motor.
     * @param power        Motor power with range of (-1 to 1)
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Inits the acquirer rotation servo to fit inside the sizing cube.
     */
    public void acquirerRotationInit() { acquirerRotation.setPosition(SERVO_INIT_POS); }

    public void acquirerRotationMid() { acquirerRotation.setPosition(0.5); }

    /**
     * Moves the acquirer rotation servo to set angle to acquire.
     */
    public void acquirerRotationSet() { acquirerRotation.setPosition(SERVO_CENTER_POS); }

    /**
     * Moves the acquirer rotation servo to set angle to acquire.
     */
    public void setAcquirerRotation(double rotations) {
        acquirerRotation.setPosition(rotations);
    }

    public double getAcquirerRotation() { return acquirerRotation.getPosition(); }

    /**
     * Rotates the acquirer using the rotation servo while the drawer slides rotates to keep the
     * minerals from falling out.
     */
    public void continuousAcquirerRotation(double distance) { }
}