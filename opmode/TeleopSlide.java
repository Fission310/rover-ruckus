package org.firstinspires.ftc.teamcode.opmode;

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
@TeleOp(name = "Teleop: Mains", group = "Teleop")
public class TeleopSlide extends OpMode {

    private static final double ANALOG_THRESHOLD = 0.2;
    private static final double SLOW_MULTIPLIER = 0.2;
    private static final double FAST_MULTIPLIER = 2.0;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareSlide robot = new HardwareSlide();

    /* Holds gamepad joystick's values */
    double yInput, xInput, slideInput;
    /* Applies slow or fast mode */
    double slowYInput, slowXInput, fastYInput, fastXInput, slowSlide, fastSlide;

    @Override
    public void init() {
    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init_loop()
     */
    @Override
    public void init_loop() {
//        robot.waitForStart();
    }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
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
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        // Adds runtime data to telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        /**
         * Gamepad1
         */
        yInput = -Range.clip(gamepad1.left_stick_y, -1.0, 1.0);
        xInput = Range.clip(gamepad1.left_stick_x, -1.0, 1.0);
        slideInput = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);

        slowYInput = Range.clip(yInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowXInput = Range.clip(xInput * SLOW_MULTIPLIER, -1.0, 1.0);
        slowSlide = Range.clip(slideInput * SLOW_MULTIPLIER, -1.0, 1.0);

        fastYInput = Range.clip(yInput * FAST_MULTIPLIER, -1.0, 1.0);
        fastXInput = Range.clip(xInput * FAST_MULTIPLIER, -1.0, 1.0);
        fastSlide = Range.clip(slideInput * FAST_MULTIPLIER, -1.0, 1.0);

        // Threshold for strafing, makes horizontal strafing easier
        if (abs(slideInput) < ANALOG_THRESHOLD) { slideInput = 0; }

        if (gamepad1.left_bumper) {
            robot.drivetrain.driveSlide(slowYInput, slowXInput, slowSlide);
            telemetry.addData("Status", "yInput: " + yInput);
            telemetry.addData("Status", "xInput: " + xInput);
        } else if (gamepad1.right_bumper) {
            robot.drivetrain.driveSlide(fastYInput, fastXInput, fastSlide);
            telemetry.addData("Status", "yInput: " + yInput);
            telemetry.addData("Status", "xInput: " + xInput);
        } else {
            robot.drivetrain.driveSlide(yInput, xInput, slideInput);
            telemetry.addData("Status", "yInput: " + yInput);
            telemetry.addData("Status", "xInput: " + xInput);
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