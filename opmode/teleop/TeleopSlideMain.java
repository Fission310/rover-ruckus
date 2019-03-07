package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.signals.BackgroundColorManager;

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
 * Left trigger:    Lowers the lift and pinion mechanism
 * Right trigger:   Extends the lift and pinion mechanism
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
@TeleOp(name = "Teleop: Main Slide", group = "Teleop")
public class TeleopSlideMain extends OpMode {

    private final double ANALOG_THRESHOLD = 0.08;
    private final double SLOW_MULTIPLIER = 0.5;
    private final double LINEAR_SLIDES_SLOW_MULTIPLIER = 0.5;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareSlide robot = new HardwareSlide();

    /* Robot controller's background manager */
    private BackgroundColorManager background = new BackgroundColorManager();

    /* Holds Gamepad 1 joystick's values */
    double yInput, xInput, slideInput;
    double slowYInput, slowXInput, slowSlide, leftTrigger1, rightTrigger1;

    /* Holds Gamepad 2 joystick's values */
    double linearSlidesInput, rotationInput;
    double acquirerIntake, leftTrigger2, rightTrigger2;

    /* Handle time complexities */
    boolean aButtonPressed, bButtonPressed, xButtonPressed, yButtonPressed, leftStickPressed;
    boolean aButton, bButton, xButton, yButton, leftStick, rightStick;

    /* Handle button positions */
    boolean drivetrainSlowMode, linearSlidesSlowMode, rotationSlowMode;

    /* Applies brake behavior */
    boolean brake = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        background.init(hardwareMap);
//        robot.imuInit(hardwareMap);

    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see OpMode#init_loop()
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status:", "Waiting to start");
        telemetry.addData("Gyro Is Calibrated", robot.imuCalibrated());
        telemetry.addData("imu angle",robot.imuAngle());
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

        /**
         * Controls the drivetrain via the left and right analog sticks || Slow mode = left stick button
         */
//        yInput = abs(gamepad1.left_stick_y) > .9 ? Math.signum(gamepad1.left_stick_y) : .9 * gamepad1.left_stick_y;
//        xInput = abs(gamepad1.right_stick_x) > .9 ? Math.signum(gamepad1.right_stick_x) : .8 * gamepad1.right_stick_x;
//        slideInput = abs(gamepad1.left_stick_x) > .9 ? Math.signum(gamepad1.left_stick_x) : .8 * gamepad1.left_stick_x;
        yInput = gamepad1.left_stick_y;
        xInput = gamepad1.right_stick_x;
        slideInput = gamepad1.left_stick_x;
        if (abs(gamepad1.left_stick_x) < ANALOG_THRESHOLD) slideInput = 0.0;

        slowYInput = Range.clip(yInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowXInput = Range.clip(xInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowSlide = Range.clip(slideInput * SLOW_MULTIPLIER, -1.0, 1.0);

        if (gamepad1.left_stick_button)
            if(!leftStickPressed) {
                drivetrainSlowMode = !drivetrainSlowMode;
                leftStickPressed = true;
            } else {}
        else leftStickPressed = false;

        if (drivetrainSlowMode) {
            background.setOrangeBackground();
            robot.drivetrain.driveSlide(slowYInput, slowXInput, slowSlide);
        } else { robot.drivetrain.driveSlide(yInput, xInput, slideInput); }

        robot.lift.setLiftPower(-gamepad1.left_trigger + gamepad1.right_trigger);
        /**
         * Gamepad 2
         */

        /**
         * Sets rotation mechanism power via the left and right triggers || Slow mode = left bumper
         */
        leftTrigger2 = abs(gamepad2.left_trigger) > .9 ? -1 : -.9 * gamepad2.left_trigger;
        rightTrigger2 = abs(gamepad2.right_trigger) > .9 ? 1 : .7 * gamepad2.right_trigger;
        rotationSlowMode = gamepad2.left_bumper;

        if (rotationSlowMode) { rotationInput = (leftTrigger2 + rightTrigger2) * .75; }
        else { rotationInput = leftTrigger2 + rightTrigger2; }

        robot.drawerSlides.setRotationPower(rotationInput);

        /**
         * Sets drawer slides power via the right joystick || Slow mode = right stick button
         */
        linearSlidesSlowMode = gamepad2.left_stick_button;
        if (linearSlidesSlowMode) {
            linearSlidesInput = abs(gamepad2.left_stick_y) > .9 ? .7 * Math.signum(gamepad2.left_stick_y) : .6 * gamepad2.left_stick_y;
        } else { linearSlidesInput = gamepad2.left_stick_y; }
        robot.drawerSlides.setDrawerSlidePower(linearSlidesInput);

        /**
         * Sets acquirer power via the right analog stick || Full power -1 to 1
         */
        acquirerIntake = gamepad2.right_stick_y;
        robot.acquirer.setIntakePower(acquirerIntake);

        /**
         * Both Gamepads
         */

//        if (gamepad1.dpad_up || gamepad2.dpad_up || gamepad1.dpad_right || gamepad2.dpad_right) {
//            acquirerRotation += .08;
//        }
//        if (gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.dpad_left || gamepad2.dpad_left) {
//            acquirerRotation -= .08;
//        }
//        acquirerRotation = Range.clip(acquirerRotation, Servo.MIN_POSITION, Servo.MAX_POSITION);
//        robot.acquirer.setAcquirerRotation(acquirerRotation);

        if (gamepad1.a || gamepad2.a)
            if(!aButtonPressed) {
                aButton = !aButton;
                aButtonPressed = true;
            } else {}
        else aButtonPressed = false;

        if (gamepad1.b || gamepad2.b)
            if(!bButtonPressed) {
                bButton = !bButton;
                bButtonPressed = true;
            } else {}
        else bButtonPressed = false;

        if (gamepad1.x || gamepad2.x)
            if(!xButtonPressed) {
                xButton = !xButton;
                xButtonPressed = true;
            } else {}
        else xButtonPressed = false;

        if (gamepad1.y || gamepad2.y)
            if(!yButtonPressed) {
                yButton = !yButton;
                yButtonPressed = true;
            } else {}
        else yButtonPressed = false;

        /**
         * Telemetry
         */
//        telemetry.addData("Slow Mode", ":" + drivetrainSlowMode);
//        double[] drawerSlides = robot.drawerSlides.getPositions();
//        double imuZAxis = robot.drivetrain.singleImu.getHeading();
//        robot.drivetrain.getDrivePower();
//        robot.drivetrain.getDriveEncoderTicks();
//
//        telemetry.addData("Rotational Arm Avg Encoder counts", "Running at %.2f",
//                robot.drawerSlides.encoderCounts());
//        telemetry.addData("Rotational Arm Encoder counts", "Running at %.2f",
//                drawerSlides[0]);
//        telemetry.addData("IMU", "Z-axis: " + imuZAxis);
    }

    @Override
    public void stop() {
        background.resetBackgroundColor();
    }
}