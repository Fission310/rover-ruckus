package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.mecanum.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.gamepad.JoystickTransform;
import org.firstinspires.ftc.teamcode.util.gamepad.StickyGamepad;
import org.firstinspires.ftc.teamcode.util.signals.BackgroundColorManager;

import static java.lang.Math.abs;

/**
 * TeleopMecanumArcade is the primary TeleOp OpMode for mecanum drivetrains. All driver-controlled actions should
 * be defined in this class.
 */
@TeleOp(name = "Teleop: Scaled Mecanum Tank [Test]", group = "Teleop")
public class TeleopMecanumScaledTank extends OpMode {

    private final double ANALOG_THRESHOLD = 0.15;
    private final double SLOW_MULTIPLIER = 0.5;
    private final double LINEAR_SLIDES_SLOW_MULTIPLIER = 0.5;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMecanum robot = new HardwareMecanum();

    private JoystickTransform transform = new JoystickTransform();
    private StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1), stickyGamepad2 = new StickyGamepad(gamepad2);

    /* Robot controller's background manager */
    private BackgroundColorManager background = new BackgroundColorManager();

    /* Holds Gamepad 1 joystick's values */
    double leftInput, rightInput, slideInput, hangerInput = 0;
    double slowYInput, slowXInput, slowSlide;

    /* Holds Gamepad 1 joystick's values */
    double cascadingSlidesInput, hopperInput, acquirerInput;

    /* Handle time complexities */
    boolean aButtonPressed, bButtonPressed, xButtonPressed, yButtonPressed, leftStickPressed, rightBumperPressed, leftBumperPressed;
    boolean aButton, bButton, xButton, yButton, leftStick, rightStick, rightBumper, leftBumper;

    /* Handle button positions */
    boolean drivetrainSlowMode;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        robot.imuInit(hardwareMap);
        background.init(hardwareMap);
        background.setGreenBackground();
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
        leftInput = gamepad1.left_stick_y;
        rightInput = gamepad1.right_stick_y;
        slideInput = -gamepad1.left_trigger + gamepad1.right_trigger;
        if (abs(slideInput) < ANALOG_THRESHOLD) slideInput = 0.0;
        robot.drivetrain.tankDriveScaled(leftInput, rightInput, slideInput);

        /**
         * Controls the Lift via the up and down dpad || Slow mode = left  button
         */
        if (gamepad1.dpad_up) hangerInput = 1;
        else if (gamepad1.dpad_down) hangerInput = -1;
        else hangerInput = 0;
        robot.lift.setLiftPower(hangerInput);

        if (stickyGamepad1.b) acquirerInput = 1;
        else acquirerInput = 0;
        robot.acquirer.setIntakePower(acquirerInput);

        if (stickyGamepad2.right_bumper) { robot.acquirer.acquirerRotationAcquirer(); }
        else { robot.acquirer.acquirerRotationDump(); }

        if (stickyGamepad2.left_bumper) { robot.hopper.hopperRotation(); }
        else { robot.hopper.hopperRotationDump(); }

        cascadingSlidesInput = gamepad2.right_stick_y;
        robot.acquirer.setCascadingSlidesPower(cascadingSlidesInput);

        hopperInput = gamepad2.left_stick_y;
        robot.hopper.setDrawerSlidePower(hopperInput);

        /**
         * Telemetry
         */
        double[] positions = robot.drivetrain.getPositions();
        telemetry.addData("Encoder counts", "lf: %.2f | rf: %.2f | lb: %.2f | rb: %.2f",
                positions[0],
                positions[1],
                positions[2],
                positions[3]);

        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    @Override
    public void stop() {
        background.resetBackgroundColor();
    }
}