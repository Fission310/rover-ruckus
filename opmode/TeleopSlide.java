package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.HardwareSlide;
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
 * Left bumper:     Decelerates slide
 * Right bumper:    Accelerates slide
 * Left trigger:    Decelerates robot
 * Right trigger:   Accelerates robot
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

    double yInput, xInput, slideInput, hangingInput, leftTrigger, rightTrigger;
    double driverXInput, driverYInput;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init_loop()
     */
    @Override
    public void init_loop() {
        robot.waitForStart();
    }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        robot.drivetrain.encoderInit();

        runtime.reset();
    }

    /**
     * Runs continuously while the OpMode is ac tive. Defines the driver-controlled actions
     * according to gamepad input.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        // Adds runtime data to telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        yInput = Range.clip(-gamepad1.left_stick_y, -1.0, 1.0);
        xInput = Range.clip(gamepad1.left_stick_x, -1.0, 1.0);
        slideInput = Range.clip(gamepad1.right_stick_x, -1.0, 1.0);
        leftTrigger = Range.clip(gamepad1.left_trigger, 0.0, 1.0);
        rightTrigger = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

        // Threshold for strafing, makes horizontal strafing easier
        if (abs(slideInput) < ANALOG_THRESHOLD) { slideInput = 0; }

        // Drives the robot based on driver joystick input
        driverYInput = Range.clip(yInput - leftTrigger + rightTrigger, -1.0, 1.0) / 1;
        driverXInput = Range.clip(xInput - leftTrigger + rightTrigger, -1.0, 1.0) / 1;

        if (gamepad1.left_bumper) {
            robot.drivetrain.driveSlide(driverYInput, driverXInput, slideInput * SLOW_MULTIPLIER);
        } else if (gamepad1.right_bumper) {
            robot.drivetrain.driveSlide(driverYInput, driverXInput, slideInput * FAST_MULTIPLIER);
        } else {
            robot.drivetrain.driveSlide(driverYInput, driverXInput, slideInput);
        }

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