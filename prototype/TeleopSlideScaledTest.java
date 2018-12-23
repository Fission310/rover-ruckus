package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.GamepadManager;

import static java.lang.Math.abs;

/**
 * TeleopMain is the primary TeleOp OpMode for slide drivetrains. All driver-controlled actions should
 * be defined in this class.
 *
 * Gamepad1 BUTTON MAPPINGS:
 * Left stick x:      Turn Robot
 * Left stick y:      Control robot's velocity and direction
 * Right stick x:     Control robot's velocity and direction (stafe)
 * Right stick y:   N/A
 * X:
 * Y:
 * A:
 * B:
 * Left bumper:     Decelerates robot
 * Right bumper:    Accelerates robot
 * Left trigger:    Decelerates slide
 * Right trigger:   Accelerates slide
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
@TeleOp(name = "Teleop: Slide Test", group = "Teleop")
@Disabled
public class TeleopSlideScaledTest extends OpMode {

    private static final double ANALOG_THRESHOLD = 0.0;
    private static final double SLOW_MULTIPLIER = 0.2;
    private static final double FAST_MULTIPLIER = 2.0;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareSlide robot = new HardwareSlide();

    /* Gamepad map */
    private GamepadManager gamepadManager1 = new GamepadManager(gamepad1);
    private GamepadManager gamepadManager2 = new GamepadManager(gamepad2);

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
    }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see OpMode#loop()
     */
    @Override
    public void start() {
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

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

        // Threshold for strafing, makes horizontal strafing easier
        if (abs(gamepadManager1.slideInput) < ANALOG_THRESHOLD) { gamepadManager1.slideInput = 0.0; }

        if (gamepad1.left_bumper) {
            robot.drivetrain.driveSlideScaled(gamepadManager1.slowXInput, gamepadManager1.slowXInput, gamepadManager1.slowSlide, gamepadManager1.right_bumper());
            telemetry.addData("Status", "slowYInput: " + gamepadManager1.slowYInput);
            telemetry.addData("Status", "slowXInput: " + gamepadManager1.slowXInput);
            telemetry.addData("Status", "slowSlide: " + gamepadManager1.slowSlide);
        } else if (gamepad1.right_bumper) {
            robot.drivetrain.driveSlideScaled(gamepadManager1.fastYInput, gamepadManager1.fastXInput, gamepadManager1.fastSlide, gamepadManager1.right_bumper());
            telemetry.addData("Status", "fastYInput: " + gamepadManager1.fastYInput);
            telemetry.addData("Status", "fastXInput: " + gamepadManager1.fastXInput);
            telemetry.addData("Status", "fastSlide: " + gamepadManager1.fastSlide);
        } else {
            robot.drivetrain.driveSlideScaled(gamepadManager1.yInput, gamepadManager1.xInput, gamepadManager1.slideInput, gamepadManager1.right_bumper());
            telemetry.addData("Status", "yInput: " + gamepadManager1.yInput);
            telemetry.addData("Status", "xInput: " + gamepadManager1.xInput);
            telemetry.addData("Status", "slideInput: " + gamepadManager1.slideInput);
        }

        /**
         * Gamepad2
         */

        double[] positions = robot.drivetrain.getPositions();
        telemetry.addData("Path2", "Running at %.2f :%.2f",
                positions[0],
                positions[1]);
    }

    @Override
    public void stop()
    {
    }
}