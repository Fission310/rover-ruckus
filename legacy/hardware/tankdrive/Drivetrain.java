package org.firstinspires.ftc.teamcode.legacy.hardware.tankdrive;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

/**
 * Drivetrain is the class that is used to define all of the hardware for a robot's drivetrain.
 * Drivetrain must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the drivetrain. <code>encoderInit()</code>
 * must be called before an autonomous action can be called.
 *
 * This class describes a tank drivetrain.
 */
public class Drivetrain extends Mechanism {

    /* Hardware members */
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private BNO055IMU imu;

    double left, right, max;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .35;
    PIDController pidRotate, pidDrive;

    /**
     * Default constructor for Drivetrain.
     */
    public Drivetrain(){ }

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
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Initialize IMU with parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.005, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
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
     * Set drivetrain motor power Arcade style based on input.
     *
     * @param y_value      power for left side of drivetrain (-1 to 1)
     * @param x_value     power for right side of drivetrain (-1 to 1)
     */
    public void driveArcade(double y_value, double x_value) {
        // Combine driveArcade and turn for blended motion.
        left = y_value - x_value;
        right = y_value + x_value;

        leftFront.setPower(left);
        leftBack.setPower(left);
        rightBack.setPower(right);
        rightFront.setPower(right);
    }

    /**
     * Set drivetrain motor power Tank style based on input.
     *
     * @param left      power for left side of drivetrain (-1 to 1)
     * @param right     power for right side of drivetrain (-1 to 1)
     */
    public void driveTank(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightBack.setPower(right);
        rightFront.setPower(right);
    }
    public void driveParametric(double v_y, double v_x, double v_rot) {
        driveTank(v_y-v_rot,v_y+v_rot);
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

        // Current heading angle of robot
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftFront.getCurrentPosition() + (int)(leftInches * Constants.TICKS_PER_INCH);
        newRightTarget = rightFront.getCurrentPosition() + (int)(rightInches * Constants.TICKS_PER_INCH);

        leftFront.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);
        leftBack.setTargetPosition(newLeftTarget);
        rightBack.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                leftFront.isBusy() && rightFront.isBusy()) {

            // Get IMU angles
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Heading angle
            double gyroAngle = angles.firstAngle;

            // Adjustment factor for heading
            double p = (gyroAngle - currentAngle) * Constants.PCONSTANT;

            // Set power of drivetrain motors accounting for adjustment
//            leftFront.setPower(Math.abs(speed) + p);
//            rightFront.setPower(Math.abs(speed) - p);
//            leftBack.setPower(Math.abs(speed) + p);
//            rightBack.setPower(Math.abs(speed) - p);
            driveStraightPID(speed, p);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition());
            opMode.telemetry.addData("Heading: ", "%f", gyroAngle);
            opMode.telemetry.addData("AccX: ", "%f", imu.getAcceleration().xAccel);
            opMode.telemetry.addData("AccY: ", "%f", imu.getAcceleration().yAccel);
            opMode.telemetry.update();
        }

        // Stop all motion
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drive accurately using a PID loop.
     * @param speed         speed at which the motor shall turn
     * @param correction         a placeholder value for PID output in case correction isnt given a new value
     */
    public void driveStraightPID(double speed, double correction) {
        double corrections = pidDrive.performPID(getAngle());

        leftFront.setPower(-speed + corrections);
        rightFront.setPower(-speed);
        leftBack.setPower(-speed + corrections);
        rightBack.setPower(-speed);
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
     * @param angle         number of degrees to turn
     * @param timeoutS      amount of time before the move should stop
     */
    public void turn(double angle, double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opMode.opModeIsActive() && Math.abs(getError(angle)) > 1.5 && runtime.seconds() < timeoutS) {

            double velocity = getError(angle) / 180 + 0.02; // this works

            // Set motor power according to calculated angle to turn
            leftFront.setPower(velocity);
            rightFront.setPower(-velocity);
            leftBack.setPower(velocity);
            rightBack.setPower(-velocity);

            // Display heading for the driver
            opMode.telemetry.addData("Heading: ", "%.2f : %.2f", angle, getHeading());
            opMode.telemetry.addData("Velocity: ", "%.2f", velocity);
            opMode.telemetry.update();
        }

        // Stop motor movement
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private double getError(double targetAngle) {
        double heading = getHeading();
        if (targetAngle > heading) {
            if (targetAngle - heading > 180) {
                return 360 - Math.abs(targetAngle) - Math.abs(heading);
            } else {
                return targetAngle - heading;
            }
        } else {
            if (targetAngle - heading > 180) {
                return -(360 - Math.abs(targetAngle) - Math.abs(heading));
            } else {
                return heading - targetAngle;
            }
        }
    }

    public double[] getPositions() {
        double[] positions = new double[4];
        positions[0] = leftFront.getCurrentPosition() / Constants.TICKS_PER_INCH;
        positions[1] = rightFront.getCurrentPosition() / Constants.TICKS_PER_INCH;

        return positions;
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
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
        resetAngle();

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
            while (opMode.opModeIsActive() && getAngle() == 0) {
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                opMode.sleep(100);
            } do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
            } while (opMode.opModeIsActive() && !pidRotate.onTarget());
        } else do { // left turn.
            power = pidRotate.performPID(getAngle()); // power will be + on left turn.
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftBack.setPower(power);
            rightBack.setPower(-power);
        } while (opMode.opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // wait for rotation to stop.
        opMode.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}