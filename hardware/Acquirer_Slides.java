package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Acquirer_Slides is the class that is used to define all of the hardware for a robot's acquirer + cascading slides.
 * Acquirer_Slides must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the acquiring process.
 */
public class Acquirer_Slides extends Mechanism {

    /* CONSTANTS */
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;

    /* Hardware members */
    private DcMotor cascadingSlides;
    private DcMotor intakeMotor;
    private Servo acquirerRotation;

    /**
     * Default constructor for Acquirer_Slides.
     */
    public Acquirer_Slides(){ }

    /**
     * Overloaded constructor for Acquirer_Slides. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Acquirer_Slides(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes acquirer hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars
        cascadingSlides = hwMap.dcMotor.get(RCConfig.CASCADING_SLIDES);
        intakeMotor = hwMap.dcMotor.get(RCConfig.INTAKE_MOTOR);
        acquirerRotation = hwMap.servo.get(RCConfig.ACQUIRER_ROTATION);

        // Set polarity
        cascadingSlides.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        cascadingSlides.setPower(0);
        intakeMotor.setPower(0);
        encoderInit();
    }

    public void encoderInit() {
        cascadingSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascadingSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets power for intake motor.
     * @param power        Motor power with range of (-1 to 1)
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public void setCascadingSlidesPower(double power) {
        cascadingSlides.setPower(power);
    }

    /**
     * Inits the acquirer rotation servo to fit inside the sizing cube.
     */
    public void acquirerRotationDump() { acquirerRotation.setPosition(MAX_POS); }

    public void acquirerRotationMid() { acquirerRotation.setPosition(MAX_POS / 2); }

    /**
     * Moves the acquirer rotation servo to set angle to acquire.
     */
    public void acquirerRotationAcquirer() { acquirerRotation.setPosition(MIN_POS); }

    /**
     * Moves the acquirer rotation servo to set angle.
     */
    public void setAcquirerRotation(double rotations) {
        acquirerRotation.setPosition(rotations);
    }

    public double getAcquirerRotation() { return acquirerRotation.getPosition(); }
}