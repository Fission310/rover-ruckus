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
    private DcMotor LiftMotor;

    /**
     * Default constructor for Acquirer.
     */
    public Lift() { }

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
        // Retrieve motor from hardware map and assign to instance vars
        LiftMotor = hwMap.dcMotor.get(RCConfig.LIFT);

        // Set braking behavior
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set initial power
        LiftMotor.setPower(0);
    }

    /**
     * Initializes motors for encoder hanging. Must be called before calling methods that use
     * encoders.
     */
    public void encoderInit() {
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets motors zero power behavior. Indicate whether the lift and pinion lift should be in float or brake mode.
     * Float mode allows for free rotations after the initial lift.
     * Brake mode prevents extra rotations after the initial lift.
     * @param behavior FLOAT, BRAKE
     */
    public void setDriveZeroPowers(DcMotor.ZeroPowerBehavior behavior) {
        LiftMotor.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets power for lift motor.
     */
    public void setLiftPower(double power) {
            LiftMotor.setPower(power);
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
        int leftTarget;

        // Determine new target position, and pass to motor controller
        leftTarget = LiftMotor.getCurrentPosition() + (int)(inches * Constants.TICKS_PER_INCH_53);

        LiftMotor.setTargetPosition(leftTarget);

        // Turn On RUN_TO_POSITION
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                LiftMotor.isBusy()) {

            // Set power of lift and pinion motors accounting for adjustment
            setLiftPower(speed);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d", leftTarget);
            opMode.telemetry.update();
        }

        // Stop all motion
        LiftMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double[] getPositions() {
        double[] positions = new double[1];
        positions[0] = LiftMotor.getCurrentPosition();

        return positions;
    }

    public double getLeftPower() {
        return LiftMotor.getPower();
    }
}