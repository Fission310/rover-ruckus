package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private static final double SERVO_CENTER_POS = 0.5;

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
        intakeMotor = hwMap.crservo.get(RCConfig.ACQUIRER_INTAKE);
        acquirerFloor = hwMap.servo.get(RCConfig.ACQUIRER_FLOOR);

        // Set polarity
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        intakeMotor.setPower(0);

        acquirerFloorInit();
    }

    /**
     * Sets power for intake motor.
     * @param power        Motor power with range of (-1 to 1)
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Inits the acquirer floor servo to not allow gold cubes to pass.
     */
    public void acquirerFloorInit() {
        acquirerFloor.setPosition(SERVO_INIT_POS);
    }

    /**
     * Moves the acquirer floor servo to allow gold cubes to pass.
     */
    public void acquirerFloorBlock() {
        acquirerFloor.setPosition(SERVO_CENTER_POS);
    }

}