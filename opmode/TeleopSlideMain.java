package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.sensors.IMU;

import static java.lang.Math.abs;

/**
 * TeleopMain is the primary TeleOp OpMode for slide drivetrains. All driver-controlled actions should
 * be defined in this class.
 *
 * Gamepad1 BUTTON MAPPINGS:
 * Left stick x:      Control the robot's slide wheel
 * Left stick y:      Control robot's velocity and direction
 * Right stick x:     Control robot's turn
 * Right stick y:   N/A
 * X:
 * Y:
 * A:
 * B:
 * Left bumper:     Decelerates robot (slow factor)
 * Right bumper:    Activates the robots zero brake behavior
 * Left trigger:
 * Right trigger:
 * DPAD_UP:
 * DPAD_DOWN:
 * DPAD_LEFT:
 * DPAD_RIGHT:
 * START:
 * BACK:
 *
 * Gamepad2 BUTTON MAPPINGS:
 * Left stick x:
 * Left stick y:
 * Right stick x:
 * Right stick y:
 * X:
 * Y:
 * A:
 * B:
 * Left bumper:
 * Right bumper:
 * Left trigger:    Activates acquirer reverse
 * Right trigger:   Activates acquirer inward
 * DPAD_UP:
 * DPAD_DOWN:
 * DPAD_LEFT:
 * DPAD_RIGHT:
 * START:
 * BACK:
 *
 */
@TeleOp(name = "Teleop: Main Slide", group = "Teleop")
public class TeleopSlideMain extends OpMode {

    private static final double ANALOG_THRESHOLD = 0.0;
    private static final double SLOW_MULTIPLIER = 0.25;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareSlide robot = new HardwareSlide();

    /* Holds gamepad joystick's values */
    double yInput, xInput, slideInput, linearSlidesInput, rotationInput;
    /* Applies slow or fast mode */
    double slowYInput, slowXInput, slowSlide, slowLinearSlidesInput, slowRotationInput, leftTrigger, rightTrigger;
    /* Applies brake behavior */
    boolean brake = false;

    @Override
    public void init() {
    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see OpMode#init_loop()
     */
    @Override
    public void init_loop() {
//        robot.waitForStart();
        telemetry.addData("Status:", "Waiting to start");
        telemetry.update();
    }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see OpMode#loop()
     */
    @Override
    public void start() {
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        robot.drivetrain.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();
    }

    /**
     * Runs continuously while the OpMode is active. Defines the driver-controlled actions
     * according to gamepad input.
     * @see OpMode#loop()
     */
    @Override
    public void loop() {
        // Adds runtime data to telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        /**
         * Gamepad1
         */
        // clip the input values so that the values never exceed +/- 1
        yInput = -Range.clip(gamepad1.left_stick_y, -1.0, 1.0);
        xInput = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);
        slideInput = -Range.clip(gamepad1.left_stick_x, -1.0, 1.0);

        slowYInput = Range.clip(yInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowXInput = Range.clip(xInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowSlide = Range.clip(slideInput * SLOW_MULTIPLIER, -1.0, 1.0);

        if (gamepad1.left_bumper) {
            robot.drivetrain.driveSlide(slowYInput, slowXInput, slowSlide);
            telemetry.addData("GP 1 Status", "slowYInput: " + slowYInput);
            telemetry.addData("GP 1 Status", "slowXInput: " + slowXInput);
            telemetry.addData("GP 1 Status", "slowSlide: " + slowSlide);
        } else {
            robot.drivetrain.driveSlide(yInput, xInput, slideInput);
            telemetry.addData("GP 1 Status", "yInput: " + yInput);
            telemetry.addData("GP 1 Status", "xInput: " + xInput);
            telemetry.addData("GP 1 Status", "slideInput: " + slideInput);
        }

        // Sets racks power via the left and right triggers
        leftTrigger = gamepad1.left_trigger > .9 ? -1 : -.5 * gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger > .9 ? 1 : .5 * gamepad1.right_trigger;
        robot.rack.setRackPower(leftTrigger + rightTrigger);

        // Changes the brake mode
        brake = gamepad1.right_bumper;
        if (brake == false) robot.drivetrain.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.BRAKE);
        else robot.drivetrain.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.FLOAT);

        /**
         * Gamepad2
         */
        // clip the input values so that the values never exceed +/- 1
        linearSlidesInput = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);
        rotationInput = Range.clip(gamepad2.right_stick_y, -1.0, 1.0);

        slowLinearSlidesInput = Range.clip(linearSlidesInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowRotationInput = Range.clip(rotationInput * SLOW_MULTIPLIER, -1.0, 1.0);

        if (gamepad2.left_bumper) {
            robot.acquirer.setLinearSlidePower(slowLinearSlidesInput);
            robot.acquirer.setRotationPower(slowRotationInput);
            telemetry.addData("GP 2 Status", "slowLinearSlidesInput: " + slowLinearSlidesInput);
            telemetry.addData("GP 2 Status", "slowRotationInput: " + slowRotationInput);
        } else {
            robot.acquirer.setLinearSlidePower(linearSlidesInput);
            robot.acquirer.setRotationPower(rotationInput);
            telemetry.addData("GP 2 Status", "linearSlidesInput: " + linearSlidesInput);
            telemetry.addData("GP 2 Status", "rotationInput: " + rotationInput);
        }

        double[] positions = robot.drivetrain.getPositions();
        double imu = robot.drivetrain.singleImu.getHeading();
        telemetry.addData("Path2", "Running at %.2f :%.2f",
                positions[0],
                positions[1]);
        telemetry.addData("IMU", "imu" + imu);
    }

    @Override
    public void stop()
    {
    }
}