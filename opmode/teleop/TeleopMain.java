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
 * TeleopMain is the primary TeleOp OpMode for mecanum drivetrains. All driver-controlled actions should
 * be defined in this class. This opmode should be used during the 2018-2019 Detroit World Championship.
 */
@TeleOp(name = "Main Teleop [Use for Worlds]", group = "Teleop")
public class TeleopMain extends OpMode {

    private final double ANALOG_THRESHOLD = 0.15;
    private final double SLOW_MULTIPLIER = 0.5;
    private final double LINEAR_SLIDES_SLOW_MULTIPLIER = 0.5;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMecanum robot = new HardwareMecanum();

    private JoystickTransform transform = new JoystickTransform();
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    /* Robot controller's background manager */
    private BackgroundColorManager background = new BackgroundColorManager();

    /* Holds Gamepad 1 joystick's values */
    double leftInput, rightInput, slideInput, hangerInput;

    /* Holds Gamepad 2 joystick's values */
    double cascadingSlidesInput, hopperInput, acquirerInput;
    double acquirerPosition = 0.0, hopperPosition = 0.0, verticalPosition = 0.0, horizontalPosition = 0.0;

    @Override
    public void init() {
        /* Robot Init */
        robot.init(hardwareMap);
        robot.imuInit(hardwareMap);

        /* Background Color */
        background.init(hardwareMap);
        background.setGreenBackground();

        /* Gamepad init */
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
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
        /* Adds runtime data to telemetry */
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
        if (stickyGamepad1.right_stick_button) {
            robot.drivetrain.fieldCentric(leftInput, gamepad1.left_stick_x, gamepad1.right_stick_x, robot.imuAngle());
        } else {
            robot.drivetrain.tankDriveScaled(leftInput, rightInput, slideInput);
        }

        /**
         * Controls the Lift via the up and down dpad || Slow mode = left  button
         */
        if (gamepad1.dpad_up) hangerInput = 1;
        else if (gamepad1.dpad_down) hangerInput = -1;
        else hangerInput = 0;
        robot.lift.setLiftPower(hangerInput);

        /**
         * Gamepad 2
         */

        /**
         * Controls the Acquirer speed via the right bumper
         */
        acquirerInput = stickyGamepad2.right_bumper ? 1 : 0;
        robot.acquirer.setIntakePower(acquirerInput);

        /**
         * Controls the Acquirer slides via the right analog stick
         */
        cascadingSlidesInput = gamepad2.right_stick_y;
        robot.acquirer.setCascadingSlidesPower(cascadingSlidesInput);

        /**
         * Rotates the Acquirer via the right trigger
         */
        robot.acquirer.setAcquirerRotation(gamepad2.right_trigger);

        /**
         * Controls the Hopper slides via the left analog stick
         */
        hopperInput = gamepad2.left_stick_y;
        robot.hopper.setDrawerSlidePower(hopperInput);

        /**
         * Rotates the Hopper via the left trigger
         */
        robot.hopper.setHopperRotation(gamepad2.left_trigger);

        /**
         * Telemetry
         */
        /* Drivetrain's encoder */
        double[] positions = robot.drivetrain.getPositions();
        /* Acquirer and Hopper servo's position */
        acquirerPosition = robot.acquirer.getAcquirerRotation();
        hopperPosition = robot.hopper.getHopperRotation();
        /* Gimbal's servo's position */
        verticalPosition = robot.gimbal.getVerticalPosition();
        horizontalPosition = robot.gimbal.getHorizontalPosition();

        telemetry.addData("Encoder counts", "lf: %.2f | rf: %.2f | lb: %.2f | rb: %.2f",
                positions[0],
                positions[1],
                positions[2],
                positions[3]);
        telemetry.addData("Acquirer Position", acquirerPosition);
        telemetry.addData("Hopper Position", hopperPosition);
        telemetry.addData("Gimbal Positions", "Vert: %.2f | Horiz: %.2f", verticalPosition, horizontalPosition);

        stickyGamepad1.update();
        stickyGamepad2.update();
        robot.updateSubsystems();
    }

    @Override
    public void stop() {
        background.resetBackgroundColor();
    }
}