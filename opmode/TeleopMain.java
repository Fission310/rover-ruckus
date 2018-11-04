package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import static java.lang.Math.abs;

/**
 * TeleopMain is the primary TeleOp OpMode for Relic Recovery. All driver-controlled actions should
 * be defined in this class.
 *
 * Gamepad1 BUTTON MAPPINGS:
 * Left stick x:      Control robot's velocity and direction (stafe)
 * Left stick y:      Control robot's velocity and direction
 * Right stick x:     Turn Robot
 * Right stick y:   N/A
 * X:               Un-extends sweeper arm
 * Y:               Extends sweeper arm
 * A:               Un-extends drop arm
 * B:               Extends drop arm
 * Left bumper:     Decelerates robot
 * Right bumper:    Accelerates robot
 * Left trigger:    Special CV program: drive to lander - drives slightly left of the middle of the lander + lifts tape measure
 * Right trigger:   Special CV program: Orient robot facing parallel to each wall - press multiple times to orient
 * DPAD_UP:         N/A
 * DPAD_DOWN:       N/A
 * DPAD_LEFT:       N/A
 * DPAD_RIGHT:      N/A
 * START:           N/A
 * BACK:            N/A
 *
 * Gamepad2 BUTTON MAPPINGS:
 * Left stick x:      Rotates linear slides
 * Left stick y:      Raise & Drops linear slides
 * Right stick x:   N/A
 * Right stick y:   Raise and drops tape measure mechanism
 * X:               Un-extends sweeper arm
 * Y:               Extends sweeper arm
 * A:               Un-extends drop arm
 * B:               Extends drop arm
 * Left bumper:     Activates acquirer reverse
 * Right bumper:    Activates acquirer inward
 * Left trigger:    N/A
 * Right trigger:   N/A
 * DPAD_UP:         N/A
 * DPAD_DOWN:       N/A
 * DPAD_LEFT:       N/A
 * DPAD_RIGHT:      N/A
 * START:           N/A
 * BACK:            N/A
 *
 */
@TeleOp(name = "Teleop: Main", group = "Teleop")
public class TeleopMain extends OpMode {

    private static final double ANALOG_THRESHOLD = 0.2;
    private static final double SLOW_MULTIPLIER = 0.5;
    private static final double FAST_MULTIPLIER = 0.5;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMain robot = new HardwareMain();

    @Override
    public void init() {
    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init_loop()
     */
    @Override
    public void init_loop() {
    }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        robot.servoArm.armUp();
        robot.servoArm.sweeperNeutral();

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

        // Threshold for strafing, makes horizontal strafing easier
        double yInput = gamepad1.left_stick_y;
        if (abs(gamepad1.left_stick_y) < ANALOG_THRESHOLD) {
            yInput = 0;
        }

        // Drives the robot based on driver joystick input, check for slow mode
        if (gamepad1.left_bumper) {
            robot.drivetrain.drive(gamepad1.left_stick_y * SLOW_MULTIPLIER, yInput * SLOW_MULTIPLIER, gamepad1.right_stick_y * SLOW_MULTIPLIER);
        } else {
            robot.drivetrain.drive(gamepad1.left_stick_y, yInput, gamepad1.right_stick_x);
        }

        // Set arms position
        if (gamepad1.x || gamepad2.x) {
            robot.servoArm.sweeperLeft();
        } else if (gamepad1.y || gamepad2.y) {
            robot.servoArm.sweeperRight();
        } else if (gamepad1.a || gamepad2.a) {
            robot.servoArm.armDown();
        } else if (gamepad1.b || gamepad2.b) {
            robot.servoArm.armUp();
        }

        double[] positions = robot.drivetrain.getPositions();
        telemetry.addData("Path2", "Running at %.2f :%.2f",
                positions[0],
                positions[1]);

    }
}