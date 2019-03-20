package org.firstinspires.ftc.teamcode.hardware.mecanum;


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
import org.firstinspires.ftc.teamcode.util.sensors.imu.SingleIMU;

/**
 * Mecanum Drivetrain is the class that is used to define all of the hardware for a robot's mecanum drivetrain.
 * Drivetrain must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the drivetrain. <code>encoderInit()</code>
 * must be called before an autonomous action can be called.
 *
 * This class describes a mecanum drivetrain (Two motor for both sides of robot + One motor for center wheels).
 */
public class Drivetrain extends Mechanism {

    /* Hardware members */
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public PIDController pidRotate, pidDrive;
    public SingleIMU singleImu = new SingleIMU();

    private double power = .35;
    private double turningPower = .50;

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
        leftFront = hwMap.dcMotor.get(RCConfig.LEFT_FRONT);
        leftBack = hwMap.dcMotor.get(RCConfig.LEFT_BACK);
        rightFront = hwMap.dcMotor.get(RCConfig.RIGHT_FRONT);
        rightBack = hwMap.dcMotor.get(RCConfig.RIGHT_BACK);

        // Set motor direction (AndyMark configuration)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor brake behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * Initializes motors for encoder driving. Must be called before calling methods that use
     * encoders.
     */
    public void encoderInit() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        pidRotate = new PIDController(0.000, 0.000, 0.000);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.0, .0, .0);
    }

    public double imuAngle() { return singleImu.getAngle(); }

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
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    /**
     *
     *
     */




    /**
     * Set drivetrain motor power based on input.
     *
     * @param x         x component of drive vector
     * @param y         y component of drive vector
     * @param turn      turn vector
     */
    public void driveTrig(double x, double y, double turn) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double v1 = r * Math.cos(robotAngle) + turn;
        double v2 = r * Math.sin(robotAngle) - turn;
        double v3 = r * Math.sin(robotAngle) + turn;
        double v4 = r * Math.cos(robotAngle) - turn;

        leftFront.setPower(v3); // v2
        leftBack.setPower(v1); // v4
        rightBack.setPower(v2); // v3
        rightFront.setPower(v4); // v1
    }

    public void driveVector(double y, double x, double turn) {
        double flPower = y + x + turn;
        double frPower = y - x - turn;
        double blPower = y - x + turn;
        double brPower = y + x - turn;

        leftFront.setPower(Range.clip(flPower,-1,1));
        leftBack.setPower(Range.clip(blPower,-1,1));
        rightBack.setPower(Range.clip(brPower,-1,1));
        rightFront.setPower(Range.clip(frPower,-1,1));
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

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftFront.getCurrentPosition() + (int)(leftInches * Constants.TICKS_PER_INCH_30);
        newRightTarget = rightFront.getCurrentPosition() + (int)(rightInches * Constants.TICKS_PER_INCH_30);
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
    public void turnPID(int angle) { pidDriveRotate(angle, turningPower); }

    public void turn180PID(int angle) { pidRotate180(angle, turningPower); }

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

        // wait for rotation to stop.
        opMode.sleep(500);

        // reset angle tracking on new heading.
        singleImu.resetAngle();
    }

    private void pidRotate180(int degrees, double power) {
        singleImu.resetAngle();

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

        // wait for rotation to stop.
        opMode.sleep(500);

        // reset angle tracking on new heading.
        singleImu.resetAngle();
    }



    public void strafeToPos(double speed, double inches, double timeoutS) {
        // Target position variables
        int newTarget;

        // Current heading angle of robot
        double currentAngle = singleImu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

//        // Determine new target position, and pass to motor controller
//        newTarget = slideDrive.getCurrentPosition() + (int)(inches * Constants.TICKS_PER_INCH_30);
//        slideDrive.setTargetPosition(newTarget);
//
//        // Turn On RUN_TO_POSITION
//        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        //test stuff to add to git
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
//                slideDrive.isBusy()) {

            // Get IMU angles
            Orientation angles = singleImu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Heading angle
            double gyroAngle = angles.firstAngle;

            //added new shit

            // Adjustment factor for heading
            double p = (gyroAngle - currentAngle) * Constants.PCONSTANT;

            // Set power of drivetrain motors accounting for adjustment
//            slideDrive.setPower(speed);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d", newTarget);
            opMode.telemetry.addData("Path2", "Running at %7d",
//                    slideDrive.getCurrentPosition());
            opMode.telemetry.addData("Heading: ", "%f", gyroAngle);
            opMode.telemetry.update();
        }
//
//        // Stop all motion
//        slideDrive.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

    public void curveTurn(int counts, double speed, double turnFraction, boolean turnRight) {
        double reducedSpeed = speed * (1 - turnFraction);
        double reducedCount = counts * (1- turnFraction);

        if (turnRight) {
            leftFront.setTargetPosition(counts);
            leftFront.setPower(speed);
            rightFront.setTargetPosition((int)reducedCount);
            rightFront.setPower(reducedSpeed);
        }
        else {
            leftFront.setTargetPosition((int)reducedCount);
            leftFront.setPower(reducedSpeed);
            rightFront.setTargetPosition(counts);
            rightFront.setPower(speed);
        }
    }


    public double[] getPositions() {
        double[] positions = new double[3];
        positions[0] = leftFront.getCurrentPosition() / Constants.TICKS_PER_INCH_30;
        positions[1] = rightFront.getCurrentPosition() / Constants.TICKS_PER_INCH_30;
//        positions[2] = slideDrive.getCurrentPosition() / Constants.TICKS_PER_INCH_30;

        return positions;
    }
}