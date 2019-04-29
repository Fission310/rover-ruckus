package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    private DcMotorEx cascadingSlides;
//    private DcMotorEx intakeMotor;
    private  CRServo intakeMotor;
    public ServoImplEx acquirerRotation;

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
        cascadingSlides = hwMap.get(DcMotorEx.class, RCConfig.CASCADING_SLIDES);
//        intakeMotor = hwMap.get(DcMotorEx.class, RCConfig.INTAKE_MOTOR);
        intakeMotor = hwMap.get(CRServo.class, RCConfig.INTAKE_MOTOR);
        acquirerRotation = hwMap.get(ServoImplEx.class, RCConfig.ACQUIRER_ROTATION);
//        acquirerRotation.setPwmRange(new PwmControl.PwmRange(880,2200));
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
//        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets power for intake motor.
     * @param power        Motor power with range of (-1 to 1)
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

//    public double getAcquirerIntakeTicks() { return intakeMotor.getCurrentPosition() * Constants.INCHES_PER_TICK_26; }

    public void setCascadingSlidesPower(double power) {
        cascadingSlides.setPower(power);
    }

    public double getAcquirerSlidesTicks() { return cascadingSlides.getCurrentPosition() * Constants.INCHES_PER_TICK_ACQUIRER; }

    public void acquirerSlideToPos(double speed, double inches, double timeoutS) {
        // Target position variables
        int newDistanceTarget;

        // Determine new target position, and pass to motor controller
        newDistanceTarget = cascadingSlides.getCurrentPosition() + (int)(inches / Constants.INCHES_PER_TICK_ACQUIRER);
        cascadingSlides.setTargetPosition(newDistanceTarget);

        // Turn On RUN_TO_POSITION
        cascadingSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cascadingSlides.setPower(Math.abs(speed));

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) && (cascadingSlides.isBusy())) {
            // Display it for the driver.
            opMode.telemetry.addData("Path1",  "Running to %7d", newDistanceTarget);
            opMode.telemetry.addData("Path2",  "Running at %7d", cascadingSlides.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop all motion
        cascadingSlides.setPower(0);

        // Turn off RUN_TO_POSITION
        cascadingSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascadingSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Inits the acquirer rotation servo to fit inside the sizing cube.
     */
    public void acquirerRotationDump() { acquirerRotation.setPosition(MAX_POS); }

    public void acquirerRotationMid() { acquirerRotation.setPosition((MAX_POS +MIN_POS)/ 2); }

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