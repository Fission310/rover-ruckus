package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Lift is the class that is used to define all of the hardware for a robot's hanging mechanism.
 * Lift must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving hanging.
 */
public class Lift extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;

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
        leftLiftMotor = hwMap.dcMotor.get(RCConfig.LEFT_HANGER);
        rightLiftMotor = hwMap.dcMotor.get(RCConfig.RIGHT_HANGER);

        // Set braking behavior
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
    }

    /**
     * Initializes motors for encoder hanging. Must be called before calling methods that use
     * encoders.
     */
    public void encoderInit() {
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets motors zero power behavior. Indicate whether the lift and pinion lift should be in float or brake mode.
     * Float mode allows for free rotations after the initial lift.
     * Brake mode prevents extra rotations after the initial lift.
     * @param behavior FLOAT, BRAKE
     */
    public void setDriveZeroPowers(DcMotor.ZeroPowerBehavior behavior) {
        rightLiftMotor.setZeroPowerBehavior(behavior);
        leftLiftMotor.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets power for lift motor.
     */
    public void setLiftPower(double power) {
            leftLiftMotor.setPower(power);
            rightLiftMotor.setPower(power);
    }

    /**
     * Scales input from the joystick of the gamepad. This allows for easier control from a range of
     * -1 to 1.
     * @param joystickValue This should be the respected joystick from the gamepad
     * @return scaled value=
     */
    double setLiftScaledPower(double joystickValue) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
        int index = (int)(joystickValue * 16.0);

        if (index < 0) index = -index;
        else if (index > 16) index = 16;

        double joystickScale = 0.0;

        if (joystickValue < 0) joystickScale = -scaleArray[index];
        else joystickScale = scaleArray[index];

        return joystickScale;
    }

    /**
     * Lift to a relative position using encoders and an IMU.
     *
     * Rack and pinion will stop moving if any of three conditions occur:
     * <ul>
     *  <li>Move gets to the desired position</li>
     *  <li>Move runs out of time</li>
     *  <li>Driver stops the running OpMode</li>
     * </ul>
     *
     * @param speed         maximum power of drivetrain motors when driving
     * @param inches    number of inches to move the lift
     */
    public void liftToPos(double speed, double inches) {
        // Target position variables
        int leftTarget, rightTarget;

        // Determine new target position, and pass to motor controller
        leftTarget = leftLiftMotor.getCurrentPosition() + (int)(inches * Constants.TICKS_PER_INCH_RACK_PINION);
        rightTarget = rightLiftMotor.getCurrentPosition() + (int)(inches * Constants.TICKS_PER_INCH_RACK_PINION);
        // Average out any differences (if any)
        int target = (leftTarget + rightTarget) / 2;

        leftLiftMotor.setTargetPosition(target);
        rightLiftMotor.setTargetPosition(target);

        // Turn On RUN_TO_POSITION
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                leftLiftMotor.isBusy() && rightLiftMotor.isBusy()) {

            // Set power of lift and pinion motors accounting for adjustment
            setLiftPower(speed);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", leftTarget, rightTarget);
            opMode.telemetry.addData("Path2 Encoder Values", "Running at %7d :%7d", leftLiftMotor.getCurrentPosition(), rightLiftMotor.getCurrentPosition());
            opMode.telemetry.addData("Lift speed/power(power is negative of speed",speed);
            opMode.telemetry.update();
        }

        // Stop all motion
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double[] getPositions() {
        double[] positions = new double[2];
        positions[0] = leftLiftMotor.getCurrentPosition();
        positions[1] = rightLiftMotor.getCurrentPosition();

        return positions;
    }

    public double getLeftPower() {
        return leftLiftMotor.getPower();
    }

    public double getRightPower() {
        return rightLiftMotor.getPower();
    }
}