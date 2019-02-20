package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * DrawerSlides is the class that is used to define all of the hardware for a robot's drawer slides.
 * DrawerSlides must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the drawer slides.
 */
public class DrawerSlides extends Mechanism {

    /* Hardware members */
    private DcMotor rotationBack;
    private DcMotor rotationForward;

    private DcMotor drawerSlide;

    /**
     * Default constructor for DrawerSlides.
     */
    public DrawerSlides(){

    }
    /**
     * Overloaded constructor for DrawerSlides. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public DrawerSlides(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes drawer slides hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars
        rotationForward = hwMap.dcMotor.get(RCConfig.FRONT_ROT);
        rotationBack = hwMap.dcMotor.get(RCConfig.BACK_ROT);
        drawerSlide = hwMap.dcMotor.get(RCConfig.DRAWER_SLIDES);

        // Set braking behavior
        rotationForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drawerSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        rotationForward.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationBack.setDirection(DcMotorSimple.Direction.REVERSE);
        drawerSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        rotationForward.setPower(0);
        rotationBack.setPower(0);
        drawerSlide.setPower(0);
    }

    public void encoderInit() {
        rotationForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drawerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotationForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drawerSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDrawerSlideZeroPowers(DcMotor.ZeroPowerBehavior behavior) {
        drawerSlide.setZeroPowerBehavior(behavior);
    }

    public void setRotationZeroPowers(DcMotor.ZeroPowerBehavior behavior) {
        rotationForward.setZeroPowerBehavior(behavior);
        rotationBack.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets power for rotation motors.
     * @param power        Motor power with range of (-1 to 1)
     */
    public void setRotationPower(double power) {
        rotationForward.setPower(power);
        rotationBack.setPower(power);
    }

    public void setScaledRotationPower(double power) {
        double scaledPower = scaleInput(power);
        rotationForward.setPower(scaledPower);
        rotationBack.setPower(scaledPower);
    }

    double scaleInput(double power) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
        int index = (int)(power * 16.0);

        if (index < 0) index = -index;
        else if (index > 16) index = 16;

        double joystickScale = 0.0;

        if (power < 0) joystickScale = -scaleArray[index];
        else joystickScale = scaleArray[index];

        return joystickScale;
    }

    public void rotateToPos(double speed, double back, double forward, double timeoutS) {
        // Target position variables
        int newForwardTarget, newBackTarget;

        // Determine new target position, and pass to motor controller
        newForwardTarget = rotationForward.getCurrentPosition() + (int)(forward * Constants.TICKS_PER_INCH_53);
        newBackTarget = rotationBack.getCurrentPosition() + (int)(back * Constants.TICKS_PER_INCH_53);
        // Average out any differences (if any)
        int target = (newForwardTarget + newBackTarget) / 2;

        rotationForward.setTargetPosition(target);
        rotationBack.setTargetPosition(target);

        // Turn On RUN_TO_POSITION
        rotationForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                rotationForward.isBusy() && rotationBack.isBusy()) {

            rotationForward.setPower(speed);
            rotationBack.setPower(speed);

            // Display info for the driver.
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    rotationForward.getCurrentPosition(),
                    rotationBack.getCurrentPosition());

            opMode.telemetry.update();
        }

        // Stop all motion
        rotationForward.setPower(0);
        rotationBack.setPower(0);

        // Turn off RUN_TO_POSITION
        rotationForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets power for linear slides motors.
     * @param power        Motor power with range of (-1 to 1)
     */
    public void setDrawerSlidePower(double power) {
        drawerSlide.setPower(power);
    }

    public void setScaledDrawerSlidePower(double power) {
        double scaledPower = scaleInput(power);
        drawerSlide.setPower(scaledPower);
    }

    public double encoderCounts() {
        double newForwardTarget = rotationForward.getCurrentPosition();
        double newBackTarget = rotationBack.getCurrentPosition();
        // Average out any differences (if any)
        double target = (newForwardTarget + newBackTarget) / 2;
        return target;
    }

    public double[] getPositions() {
        double[] positions = new double[2];
        positions[0] = rotationForward.getCurrentPosition();
        positions[1] = rotationBack.getCurrentPosition();

        return positions;
    }

}