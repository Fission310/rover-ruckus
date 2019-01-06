package org.firstinspires.ftc.teamcode.hardware.slidedrive;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RCConfig;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.sensors.SingleIMU;

/**
 * Slide Drivetrain is the class that is used to define all of the hardware for a robot's slide drivetrain.
 * Drivetrain must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the drivetrain. <code>encoderInit()</code>
 * must be called before an autonomous action can be called.
 *
 * This class describes a slidedrive drivetrain (Two motor for both sides of robot + One motor for center wheels).
 */
public class Drivetrain extends Mechanism {

    /* Hardware members */
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor slideDrive;

    private BNO055IMU imu;

    double left, right, max;

    public PIDController pidRotate, pidDrive;
    public SingleIMU singleImu= new SingleIMU();

    private double power = .30;

    /**
     * Default constructor for Drivetrain.
     */
    public Drivetrain(){

    }
    /**
     * Overloaded constructor for Drivetrain. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Drivetrain(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes drivetrain hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {

        // Retrieve motors from hardware map and assign to instance vars
        leftFront = hwMap.dcMotor.get(RCConfig.LEFT_FRONT);
        rightFront = hwMap.dcMotor.get(RCConfig.RIGHT_FRONT);
        slideDrive = hwMap.dcMotor.get(RCConfig.SLIDE_DRIVE);

        // Set motor direction (AndyMark configuration)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        slideDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor brake behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        slideDrive.setPower(0);

        // Retrieve and initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        singleImu.init(imu, AxesOrder.ZYX,0D);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.006, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);
    }

    /**
     * Initializes motors for encoder driving. Must be called before calling methods that use
     * encoders.
     */
    public void encoderInit() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets motors zero power behavior. Indicate whether the drivetrain should be in float or brake mode.
     * Float mode allows for free rotations after the initial drive.
     * Brake mode prevents extra rotations after the initial drive.
     * @param behavior FLOAT, BRAKE
     */
    public void setDriveZeroPowers(DcMotor.ZeroPowerBehavior behavior) {
        rightFront.setZeroPowerBehavior(behavior);
        leftFront.setZeroPowerBehavior(behavior);
        slideDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * Scales input from the joystick of the gamepad. This allows for easier control from a range of
     * -1 to 1.
     * @param joystickValue This should be the respected joystick from the gamepad
     * @return scaled value=
     */
    double scaleInput(double joystickValue) {
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
     * Set drivetrain motor power based on input.
     *
     * @param y_value      power for vertical direction of drivetrain (-1 to 1)
     * @param x_value     power for turning of drivetrain (-1 to 1)
     */
    public void drive(double y_value, double x_value) {
        // Combine driveArcade and turn for blended motion.
        double left = Range.clip(y_value - x_value, -1.0, 1.0);
        double right = Range.clip(y_value + x_value, -1.0, 1.0);

        leftFront.setPower(left);
        rightFront.setPower(right);
    }

    /**
     * Set drivetrain motor power based on input.
     *
     * @param y_value      power for vertical direction of drivetrain (-1 to 1)
     * @param x_value     power for turning of drivetrain (-1 to 1)
     * @param slide     power for sliding of drivetrain (-1 to 1)
     */
    public void driveSlide(double y_value, double x_value, double slide) {
        // Combine driveArcade and turn for blended motion.
        double left = Range.clip(y_value - x_value, -1.0, 1.0);
        double right = Range.clip(y_value + x_value, -1.0, 1.0);

        leftFront.setPower(left);
        rightFront.setPower(right);
        strafe(slide);
    }

    /**
     * Set drivetrain motor power based on a scaled input.
     *
     * @param y_value      power for vertical direction of drivetrain (-1 to 1)
     * @param x_value     power for turning of drivetrain (-1 to 1)
     * @param slide     power for sliding of drivetrain (-1 to 1)
     */
    public void driveSlideScaled(double y_value, double x_value, double slide) {
        // Combine driveArcade and turn for blended motion.
        double left = Range.clip(y_value - x_value, -1.0, 1.0);
        double right = Range.clip(y_value + x_value, -1.0, 1.0);

        left = (double)scaleInput(left);
        right = (double)scaleInput(right);

        leftFront.setPower(left);
        rightFront.setPower(right);
        strafe((double)scaleInput(slide));
    }

    /**
     * Drive to a relative position using encoders and an IMU.
     *
     * Robot will stop moving if any of three conditions occur:
     * <ul>
     *  <li>Move gets to the desired position</li>
     *  <li>Move runs out of time</li>
     *  <li>Driver stops the running OpMode</li>
     * </ul>
     *
     * @param speed         maximum power of drivetrain motors when driving
     * @param leftInches    number of inches to move on the left side
     * @param rightInches   number of inches to move on the right side
     * @param timeoutS      amount of time before the move should stop
     */
    public void driveToPos(double speed, double leftInches, double rightInches, double timeoutS) {
        // Target position variables
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftFront.getCurrentPosition() + (int)(leftInches * Constants.TICKS_PER_INCH_MR);
        newRightTarget = rightFront.getCurrentPosition() + (int)(rightInches * Constants.TICKS_PER_INCH_MR);
        leftFront.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                leftFront.isBusy() && rightFront.isBusy()) {

            // Set power of drivetrain motors accounting for adjustment
            driveStraightPID(speed);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",

            leftFront.getCurrentPosition(),
            rightFront.getCurrentPosition());

            opMode.telemetry.addData("AccX: ", "%f", imu.getAcceleration().xAccel);
            opMode.telemetry.addData("AccY: ", "%f", imu.getAcceleration().yAccel);
            opMode.telemetry.update();
        }

        // Stop all motion
        leftFront.setPower(0);
        rightFront.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drive accurately using a PID loop.
     * @param speed         speed at which the motor shall turn
     */
    public void driveStraightPID(double speed) {
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        double corrections = pidDrive.performPID(singleImu.getAngle());

        leftFront.setPower(-speed + corrections);
        rightFront.setPower(-speed);
    }

    /**
     * Turn to a specified angle accurately using a PID loop.
     * @param angle         number of degrees to turn
     */
    public void turnPID(int angle) {
        rotate(angle, power);
    }

    /**
     * Turn to a specified angle using an IMU.
     *
     * Robot will stop moving if any of three conditions occur:
     * <li>
     *  <ol>Move gets to the desired angle</ol>
     *  <ol>Move runs out of time</ol>
     *  <ol>Driver stops the running OpMode</ol>
     * </li>
     *
     * @param speed         maximum power of drivetrain motors when driving
     * @param angle         number of degrees to turn
     * @param timeoutS      amount of time before the move should stop
     */
    public void turn(double speed, double angle, double timeoutS) {
        // Get IMU angles
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Calculate angle to turn
        double angleToTurn = angle - angles.firstAngle;

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() && Math.abs(angleToTurn) > 0.5 && runtime.seconds() < timeoutS) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleToTurn = angle - angles.firstAngle;

            // Set motor power according to calculated angle to turn
            leftFront.setPower(-Math.signum(angleToTurn) * speed);
            rightFront.setPower(Math.signum(angleToTurn) * speed);
            slideDrive.setPower(Math.signum(angleToTurn) * speed);

            // Display heading for the driver
            opMode.telemetry.addData("Heading: ", "%f", angleToTurn);
            opMode.telemetry.update();
        }

        // Stop motor movement
        leftFront.setPower(0);
        rightFront.setPower(0);
        slideDrive.setPower(0);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * Starts pid controller. PID controller will monitor the turn angle with respect to the
     * target angle and reduce power as we approach the target angle with a minimum of 20%.
     * This is to prevent the robots momentum from overshooting the turn after we turn off the
     * power. The PID controller reports onTarget() = true when the difference between turn
     * angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
     * The minimum power is determined by testing and must enough to prevent motor stall and
     * complete the turn. Note: if the gap between the starting power and the stall (minimum)
     * power is small, overshoot may still occur. Overshoot is dependant on the motor and
     * gearing configuration, starting power, weight of the robot and the on target tolerance.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        singleImu.resetAngle();

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) { // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && singleImu.getAngle() == 0) {
                leftFront.setPower(-power);
                rightFront.setPower(power);
                opMode.sleep(100);
            } do {
                power = pidRotate.performPID(singleImu.getAngle()); // power will be - on right turn.
                leftFront.setPower(power);
                rightFront.setPower(-power);
            } while (opMode.opModeIsActive() && !pidRotate.onTarget());
        } else do { // left turn.
            power = pidRotate.performPID(singleImu.getAngle()); // power will be + on left turn.
            leftFront.setPower(power);
            rightFront.setPower(-power);
        } while (opMode.opModeIsActive() && !pidRotate.onTarget());

        leftFront.setPower(0);
        rightFront.setPower(0);

        // wait for rotation to stop.
        opMode.sleep(500);

        // reset angle tracking on new heading.
        singleImu.resetAngle();
    }

    /**
     * Set center motor power for strafing.
     *
     * @param power      power for horizontal direction of drivetrain (-1 to 1)
     */
    public void strafe(double power){
        slideDrive.setPower(power);
    }

    public void strafeToPos(double speed, double inches, double timeoutS) {

        // Target position variables
        int newTarget;

        // Current heading angle of robot
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Determine new target position, and pass to motor controller
        newTarget = slideDrive.getCurrentPosition() + (int) (inches * Constants.TICKS_PER_INCH_MR);
        slideDrive.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                slideDrive.isBusy()) {

            // Get IMU angles
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Heading angle
            double gyroAngle = angles.firstAngle;

            // Adjustment factor for heading
            double p = (gyroAngle - currentAngle) * Constants.PCONSTANT;

            // Set power of drivetrain motors accounting for adjustment
            slideDrive.setPower(p);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d", newTarget);
            opMode.telemetry.addData("Path2", "Running at %7d",
                    slideDrive.getCurrentPosition());
            opMode.telemetry.addData("Heading: ", "%f", gyroAngle);
            opMode.telemetry.addData("AccX: ", "%f", imu.getAcceleration().xAccel);
            opMode.telemetry.addData("AccY: ", "%f", imu.getAcceleration().yAccel);
            opMode.telemetry.update();
        }

        // Stop all motion
        slideDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double[] getPositions() {
        double[] positions = new double[4];
        positions[0] = leftFront.getCurrentPosition() / Constants.TICKS_PER_INCH_MR;
        positions[1] = rightFront.getCurrentPosition() / Constants.TICKS_PER_INCH_MR;

        return positions;
    }
}