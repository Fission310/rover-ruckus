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
    private DcMotor rotationBack;
    private DcMotor rotationForward;

    private DcMotor linearSlide;

    private CRServo rightIntakeMotor;
    private CRServo leftIntakeMotor;

//    private Servo acquirerFloor;
    private Servo acquirerRotation;

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
        rightIntakeMotor = hwMap.crservo.get(RCConfig.LEFT_ACQUIRER_INTAKE);
        leftIntakeMotor = hwMap.crservo.get(RCConfig.RIGHT_ACQUIRER_INTAKE);
        rotationForward = hwMap.dcMotor.get(RCConfig.FRONT_ROT);
        rotationBack = hwMap.dcMotor.get(RCConfig.BACK_ROT);
        linearSlide = hwMap.dcMotor.get(RCConfig.LINEAR_SLIDES);

        //        acquirerFloor = hwMap.servo.get(RCConfig.ACQUIRER_FLOOR);
        acquirerRotation = hwMap.servo.get(RCConfig.ACQUIRER_ROTATION);

        // Set braking behavior
        rotationForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationForward.setDirection(DcMotorSimple.Direction.FORWARD);
        rotationBack.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set initial power
        rightIntakeMotor.setPower(0);
        leftIntakeMotor.setPower(0);
        rotationForward.setPower(0);
        rotationBack.setPower(0);
        linearSlide.setPower(0);
//        acquirerFloorInit();
    }

    public void encoderInit() {
        rotationForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotationForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets power for intake motor.
     * @param power        Motor power with range of (-1 to 1)
     */
    public void setIntakePower(double power) {
        double sign = Math.signum(power);
        leftIntakeMotor.setPower(.7 * sign);
        rightIntakeMotor.setPower(.7 * sign);
    }

    /**
     * Sets power for rotation motors.
     * @param power        Motor power with range of (-1 to 1)
         */
        public void setRotationPower(double power) {
            rotationForward.setPower(power);
            rotationBack.setPower(power);
        }

        /**
         * Sets power for linear slides motors.
         * @param power        Motor power with range of (-1 to 1)
         */
    public void setLinearSlidePower(double power) {
        linearSlide.setPower(power);
    }

    /**
     * Inits the acquirer floor servo to not allow gold cubes to pass.
     */
//    public void acquirerFloorInit() {
//        acquirerFloor.setPosition(SERVO_INIT_POS);
//    }


    /**
     * Moves the acquirer floor servo to allow gold cubes to pass.
     */
//    public void acquirerFloorBlock() {
//        acquirerFloor.setPosition(SERVO_CENTER_POS);
//    }

    /**
     * Inits the acquirer rotation servo to fit inside the sizing cube.
     */
    public void acquirerRotationInit() {
        acquirerRotation.setPosition(SERVO_INIT_POS);
    }

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
}