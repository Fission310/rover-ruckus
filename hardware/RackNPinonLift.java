package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RCConfig;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.sensors.SingleIMU;


/**
 * RackNPinonLift is the class that is used to define all of the hardware for a robot's hanging mechanism.
 * RackNPinonLift must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving hanging.
 */
public class RackNPinonLift extends Mechanism {

    /* CONSTANTS */

    /* Hardware members */
    private DcMotor leftRackMotor;
    private DcMotor rightRackMotor;

    private BNO055IMU imu;

    double power = .30;
    PIDController pidRackPinion;
    SingleIMU singleImu= new SingleIMU();

    public DistanceSensor sensorRange;
    /**
     * Default constructor for Acquirer.
     */
    public RackNPinonLift(){

    }
    /**
     * Overloaded constructor for Lift. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public RackNPinonLift(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes lift hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars

        // Retrieve motor from hardware map and assign to instance vars
        leftRackMotor = hwMap.dcMotor.get(RCConfig.LEFT_HANGER);
        rightRackMotor = hwMap.dcMotor.get(RCConfig.RIGHT_HANGER);

        // Set braking behavior
        leftRackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        leftRackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        leftRackMotor.setPower(0);
        rightRackMotor.setPower(0);

        // Retrieve and initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        singleImu.init(imu, AxesOrder.ZYX,0D);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidRackPinion = new PIDController(.05, 0, 0);

        // Set up parameters for driving in a straight line.
        pidRackPinion.setSetpoint(0);
        pidRackPinion.setOutputRange(0, power);
        pidRackPinion.setInputRange(-90, 90);
        pidRackPinion.enable();

        sensorRange = hwMap.get(DistanceSensor.class, RCConfig.DISTANCE_SENSOR_BOTTOM);
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
    }

    /**
     * Initializes motors for encoder hanging. Must be called before calling methods that use
     * encoders.
     */
    public void encoderInit() {
        leftRackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets power for rack motor.
     */
    public void setRackPower(double power) {
            leftRackMotor.setPower(power);
            rightRackMotor.setPower(power);
    }

    /**
     * Scales input from the joystick of the gamepad. This allows for easier control from a range of
     * -1 to 1.
     * @param joystickValue This should be the respected joystick from the gamepad
     * @return scaled value=
     */
    double setRackScaledPower(double joystickValue) {
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
     * @param inches    number of inches to move the rack
     * @param timeoutS      amount of time before the move should stop
     */
    public void rackToPos(double speed, double inches, double timeoutS) {
        // Target position variables
        int newTarget, newTargets;

        // Determine new target position, and pass to motor controller
        newTarget = leftRackMotor.getCurrentPosition() + (int)(inches * Constants.TICKS_PER_INCH_MR);
        newTargets = rightRackMotor.getCurrentPosition() + (int)(inches * Constants.TICKS_PER_INCH_MR);
        leftRackMotor.setTargetPosition(newTarget);
        rightRackMotor.setTargetPosition(newTargets);

        // Turn On RUN_TO_POSITION
        leftRackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                leftRackMotor.isBusy() && rightRackMotor.isBusy()) {

            // Set power of rack and pinion motors accounting for adjustment
            setRackPower(speed);

            // Display info for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d", newTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    leftRackMotor.getCurrentPosition(),
                    rightRackMotor.getCurrentPosition());

            opMode.telemetry.addData("AccX: ", "%f", imu.getAcceleration().xAccel);
            opMode.telemetry.addData("AccY: ", "%f", imu.getAcceleration().yAccel);
            opMode.telemetry.update();
        }

        // Stop all motion
        leftRackMotor.setPower(0);
        rightRackMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftRackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getDistanceSensor() {
        return sensorRange.getDistance(DistanceUnit.CM);
    }
}