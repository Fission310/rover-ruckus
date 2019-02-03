package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;

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
 * Right bumper:
 * Left trigger:    Lowers the rack and pinion mechanism
 * Right trigger:   Extends the rack and pinion mechanism
 * DPAD_UP:
 * DPAD_DOWN:
 * DPAD_LEFT:
 * DPAD_RIGHT:
 * START:
 * BACK:
 *
 * Gamepad2 BUTTON MAPPINGS:
 * Left stick x:
 * Left stick y:    Extends the linear slide of the acquirer
 * Right stick x:
 * Right stick y:   Rotates the linear slide mechanism
 * X:               Flips the acquirer within the robots dimensions
 * Y:
 * A:
 * B:
 * Left bumper:     Decelerates the linear slide
 * Right bumper:
 * Left trigger:    Activates acquirer reverse
 * Right trigger:   Activates acquirer inward
 * DPAD_UP:         Rotates the acquirer upwards
 * DPAD_DOWN:       Rotates the acquirer downwards
 * DPAD_LEFT:
 * DPAD_RIGHT:
 * START:
 * BACK:
 *
 */
@TeleOp(name = "Teleop: Scaled Slide", group = "Teleop")
public class TeleopSlideTest extends OpMode {

    private static final double ANALOG_THRESHOLD = 0.0;
    private static final double SLOW_MULTIPLIER = 0.4;
    private static final double LINEAR_SLIDES_SLOW_MULTIPLIER = 0.5;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareSlide robot = new HardwareSlide();

    /* Holds gamepad joystick's values */
    double yInput, xInput, slideInput; // Gamepad 1
    double linearSlidesInput, rotationInput; // Gamepad 2
    /* Applies slow or fast mode */
    double slowYInput, slowXInput, slowSlide, leftTrigger1,rightTrigger1; // Gamepad 1
    double slowLinearSlidesInput, slowRotationInput, acquirerIntake, acquirerOuttake, leftTrigger2, rightTrigger2; // Gamepad 2
    /* Handle time complexities */
    boolean aButtonPressed = false, bButtonPressed = false, xButtonPressed = false, yButtonPressed = false;
    /* Handle button positions */
    boolean left, right;
    double currentAcquirerRotation = 0;


    /* Applies brake behavior */
    boolean brake = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        robot.drivetrain.imuInit(hardwareMap);
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
         * Gamepad 1
         */
        yInput = gamepad1.left_stick_y;
        xInput = gamepad1.right_stick_x;
        slideInput = gamepad1.left_stick_x;

        slowYInput = Range.clip(yInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowXInput = Range.clip(xInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowSlide = Range.clip(slideInput * SLOW_MULTIPLIER, -1.0, 1.0);

        if (gamepad1.left_bumper) {
            robot.drivetrain.driveSlideScaled(slowYInput, slowXInput, slowSlide);
            telemetry.addData("GP 1 Status", "slowYInput: " + slowYInput);
            telemetry.addData("GP 1 Status", "slowXInput: " + slowXInput);
            telemetry.addData("GP 1 Status", "slowSlide: " + slowSlide);
        } else {
            robot.drivetrain.driveSlideScaled(yInput, xInput, slideInput);
            telemetry.addData("GP 1 Status", "yInput: " + yInput);
            telemetry.addData("GP 1 Status", "xInput: " + xInput);
            telemetry.addData("GP 1 Status", "slideInput: " + slideInput);
        }

        leftTrigger1 = Math.abs(gamepad1.left_trigger) > .9 ? -1 * Math.signum(gamepad1.left_trigger) : -.8 * gamepad1.left_trigger;
        rightTrigger1 = Math.abs(gamepad1.right_trigger) > .9 ? 1 * Math.signum(gamepad1.right_trigger) : .8 * gamepad1.right_trigger;
//        robot.rack.setRackPower(leftTrigger1 + rightTrigger1);

        /**
         * Gamepad 2
         */
//      Sets rotation mechanism power via the left and right triggers
        leftTrigger2 = gamepad2.left_trigger;
        rightTrigger2 = gamepad2.right_trigger;
        robot.drawerSlides.setScaledRotationPower(leftTrigger2 + rightTrigger2);

//      Sets drawer slides power via the right joystick
        linearSlidesInput = gamepad2.right_stick_y;
        robot.drawerSlides.setScaledDrawerSlidePower(linearSlidesInput);

//        /**
//         * Both Gamepads
//         */
        if (gamepad1.x || gamepad2.x) {
            if(!xButtonPressed) {
                robot.marker.markerLeft();
                xButtonPressed = !xButtonPressed;
            } else {
                robot.marker.markerRight();
                xButtonPressed = !xButtonPressed;
            }
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        double[] positions = robot.drivetrain.getPositions();
//        double[] rackPositions = robot.rack.getPositions();
        double imuZAxis = robot.drivetrain.singleImu.getZAxis();
        telemetry.addData("Encoder counts", "Running at %.2f :%.2f :%.2f",
                positions[0],
                positions[1],
                positions[2]);
//        telemetry.addData("Rack & Pinion counts", "Running at %.2f :%.2f",
//                rackPositions[0],
//                rackPositions[1]);
        telemetry.addData("IMU", "Z-axis: " + imuZAxis);
    }

    @Override
    public void stop() { }
}