package org.firstinspires.ftc.teamcode.hardware.slidedrive;


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
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RCConfig;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.util.motion.PIDController;
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

    private static final double WHEEL_BASE = 15;

    /* Hardware members */
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor slideDrive;

    public PIDController pidRotate, pidDrive;
    public SingleIMU singleImu = new SingleIMU();

    private double power = .60;

    /**
     * Default constructor for Drivetrain.
     */
    public Drivetrain() { }
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
        leftFront = hwMap.dcMotor.get(RCConfig.LEFT_DRIVE);
        rightFront = hwMap.dcMotor.get(RCConfig.RIGHT_DRIVE);
        slideDrive = hwMap.dcMotor.get(RCConfig.SLIDE_DRIVE);

        // Set motor direction (AndyMark configuration)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        slideDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor brake behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        slideDrive.setPower(0);
    }

    public void inits(HardwareMap hwMap) {
        // Retrieve motors from hardware map and assign to instance vars
        try {
            leftFront = hwMap.dcMotor.get(RCConfig.LEFT_DRIVE);
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setPower(0);
        } catch (Exception e) {
            opMode.telemetry.addData("Failed to init", "Left: ");
        }

        try {
            rightFront = hwMap.dcMotor.get(RCConfig.RIGHT_DRIVE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setPower(0);
        } catch (Exception e) {
            opMode.telemetry.addData("Failed to init", "Right: ");
        }

        try {
            slideDrive = hwMap.dcMotor.get(RCConfig.SLIDE_DRIVE);
            slideDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            slideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideDrive.setPower(0);
        } catch (Exception e) {
            opMode.telemetry.addData("Failed to init", "Slide: ");
        }
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
     * Initializes imu for accurate driving. Must be called after landing to get accurate
     * readings.
     */
    public void imuInit(HardwareMap hwMap) {
        // Retrieve and initialize the IMU
        singleImu.init(hwMap, AxesOrder.ZYX,0D);
        // Set the starting angle to make automating hanging easier

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.0025, 0.002, .0025);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.1, .0, .0);
    }

    public boolean imuCalibrated() {
        return singleImu.imuCalibrated();
    }

    public double imuAngle() {
        return singleImu.getAngle();
    }

    public void imuStartingRot() {
        singleImu.setStartingAngle();
    }

    public void resetDeltaAngle() {
        singleImu.resetStartingAngle();
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

    public void setSlideDriveZeroPower(DcMotor.ZeroPowerBehavior behavior) {
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

        left = scaleInput(left);
        right = scaleInput(right);

        leftFront.setPower(left);
        rightFront.setPower(right);
        strafe(scaleInput(slide));
    }

    /**
     * Drive to a relative position using encoders and an IMU. Note (You must pass in the same
     * distance for both left and right inches for this method to work correctly)
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
//      int avgTarget;
        // Determine new target position, and pass to motor controller
        newLeftTarget = leftFront.getCurrentPosition() + (int)(leftInches * Constants.TICKS_PER_INCH_30);
        newRightTarget = rightFront.getCurrentPosition() + (int)(rightInches * Constants.TICKS_PER_INCH_30);
//        avgTarget = (int)(newLeftTarget + newRightTarget);
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
            driveStraightPID(speed, leftInches);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
            leftFront.getCurrentPosition(),
            rightFront.getCurrentPosition());

            opMode.telemetry.update();
        }

        // Stop all motion
        leftFront.setPower(0);
        rightFront.setPower(0);
        setSlideDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveToPos(double speed, double distance, double timeoutS) {
        driveToPos(speed, distance, distance, timeoutS);
    }

    /**
     * Drive accurately using a PID loop.
     * @param speed         speed at which the motor shall turn
     */
    public void driveStraightPID(double speed, double distance) {
        double leftSpeed = -speed, rightSpeed = -speed;

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        double corrections = pidDrive.performPID(singleImu.getAngle());

        if (Math.signum(distance) >= 0) {
            leftFront.setPower(leftSpeed + corrections);
            rightFront.setPower(rightSpeed);
        } else if (Math.signum(distance) < 0){
            leftFront.setPower(leftSpeed);
            rightFront.setPower(rightSpeed + Math.signum(speed) * corrections);
        }
    }

    /**
     * Turn to a specified angle accurately using a PID loop.
     * - positive is counterclockwise
     * - negative is clockwise
     * @param angle         number of degrees to turn
     */
    public void turnPID(int angle) { pidDriveRotate(angle, power); }

    public void turn180PID(int angle) { pidRotate180(angle, power); }

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
    private void pidDriveRotate(int degrees, double power) {
        singleImu.resetAngle();
        setSlideDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.2, power);
        pidRotate.setTolerance(.7);
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
        setSlideDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for rotation to stop.
        opMode.sleep(500);

        // reset angle tracking on new heading.
        singleImu.resetAngle();
    }

    private void pidRotate180(int degrees, double power) {
        singleImu.resetAngle();
        setSlideDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 180);
        pidRotate.setOutputRange(.2, power);
        pidRotate.setTolerance(.8);
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
        setSlideDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for rotation to stop.
        opMode.sleep(500);

        // reset angle tracking on new heading.
        singleImu.resetAngle();
    }

    /**
     * Set center motor power for strafing.
     * @param power      power for horizontal direction of drivetrain (-1 to 1)
     */
    public void strafe(double power){
        slideDrive.setPower(power);
    }

    public void strafeToPos(double speed, double inches, double timeoutS) {
        // Target position variables
        int newTarget;

        // Current heading angle of robot
        double currentAngle = singleImu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Determine new target position, and pass to motor controller
        newTarget = slideDrive.getCurrentPosition() + (int)(inches * Constants.TICKS_PER_INCH_30);
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
            Orientation angles = singleImu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Heading angle
            double gyroAngle = angles.firstAngle;

            // Adjustment factor for heading
            double p = (gyroAngle - currentAngle) * Constants.PCONSTANT;

            // Set power of drivetrain motors accounting for adjustment
            slideDrive.setPower(speed);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d", newTarget);
            opMode.telemetry.addData("Path2", "Running at %7d",
                    slideDrive.getCurrentPosition());
            opMode.telemetry.addData("Heading: ", "%f", gyroAngle);
            opMode.telemetry.update();
        }

        // Stop all motion
        slideDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void odometryTurn(double speed, double radius, double angle, double timeoutS) {
        // Target position variables
        int newLeftTarget = 0;
        int newRightTarget = 0;

        if (angle < 0) {
            newLeftTarget = leftFront.getCurrentPosition() + (int)((radius + WHEEL_BASE) * Constants.TICKS_PER_INCH_30);
            newRightTarget = rightFront.getCurrentPosition() + (int)(radius * Constants.TICKS_PER_INCH_30);
        } else if (angle >= 0) {
            newLeftTarget = leftFront.getCurrentPosition() + (int)(radius * Constants.TICKS_PER_INCH_30);
            newRightTarget = rightFront.getCurrentPosition() + (int)((radius + WHEEL_BASE) * Constants.TICKS_PER_INCH_30);
        }

        // Determine new target position, and pass to motor controller
        leftFront.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);
        setSlideDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);

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
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition());

            opMode.telemetry.update();
        }

        // Stop all motion
        leftFront.setPower(0);
        rightFront.setPower(0);
        setSlideDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double[] getPositions() {
        double[] positions = new double[3];
        positions[0] = leftFront.getCurrentPosition() / Constants.TICKS_PER_INCH_30;
        positions[1] = rightFront.getCurrentPosition() / Constants.TICKS_PER_INCH_30;
        positions[2] = slideDrive.getCurrentPosition() / Constants.TICKS_PER_INCH_30;

        return positions;
    }

    public void getDrivePower(){
        opMode.telemetry.addData("GP 1 Status", "Left Input: " + leftFront.getPower());
        opMode.telemetry.addData("GP 1 Status", "Right Input: " + rightFront.getPower());
        opMode.telemetry.addData("GP 1 Status", "Slide: " + slideDrive.getPower());
    }

    public void getDriveEncoderTicks(){
        double[] positions = getPositions();
        opMode.telemetry.addData("Drivetrain Encoder", "Left: " + positions[0]);
        opMode.telemetry.addData("Drivetrain Encoder", "Right: " + positions[1]);
        opMode.telemetry.addData("Drivetrain Encoder", "Slide: " + positions[2]);
    }
}