package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public DcMotorEx liftMotor;

    /**
     * Default constructor for Acquirer_Slides.
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
        liftMotor = hwMap.get(DcMotorEx.class, RCConfig.LIFT);

        // Set braking behavior
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set initial power
        liftMotor.setPower(0);
        encoderInit();

    }

    /**
     * Initializes motors for encoder hanging. Must be called before calling methods that use
     * encoders.
     */
    public void encoderInit() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void noEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets motors zero power behavior. Indicate whether the lift and pinion lift should be in float or brake mode.
     * Float mode allows for free rotations after the initial lift.
     * Brake mode prevents extra rotations after the initial lift.
     * @param behavior FLOAT, BRAKE
     */
    public void setDriveZeroPowers(DcMotor.ZeroPowerBehavior behavior) {
        liftMotor.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets power for lift motor.
     */
    public void setLiftPower(double power) {
            liftMotor.setPower(power);
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
        int target;

        // Determine new target position, and pass to motor controller
        target = liftMotor.getCurrentPosition() + (int)(inches / LiftConstants.TICKS_PER_LINEAR_LIFT_INCH);

        liftMotor.setTargetPosition(target);

        // Turn On RUN_TO_POSITION
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        //ElapsedTime runtime = new ElapsedTime();
        //runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() && liftMotor.isBusy()) {

            // Set power of lift motor accounting for adjustment
            setLiftPower(speed);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d", target);
            opMode.telemetry.addData("current", liftMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop all motion
        liftMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftToPosJank(){
        liftMotor.setTargetPosition(12020);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
        int current;
        int past = -10000;
        while(opMode.opModeIsActive() && liftMotor.isBusy()){
            current = liftMotor.getCurrentPosition();
            if (current == past) break;
        }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void dropoPosJank(){
        liftMotor.setTargetPosition(9000);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(-1);
        while(opMode.opModeIsActive() && liftMotor.isBusy());
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getPositions() {
        return liftMotor.getCurrentPosition() / LiftConstants.TICKS_PER_LEADSCREW;
    }

    public double getLftPower() {
        return liftMotor.getPower();
    }
}