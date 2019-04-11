package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.mecanum.HardwareMecanum;
import org.firstinspires.ftc.teamcode.util.gamepad.JoystickTransform;
import org.firstinspires.ftc.teamcode.util.gamepad.StickyGamepad;
import org.firstinspires.ftc.teamcode.util.signals.BackgroundColorManager;

import static java.lang.Math.abs;

/**
 * Main Teleop is the competition ready TeleOp OpMode for the mecanum drivetrain. All driver-controlled
 * actions should be defined in this class.
 */
@TeleOp(name = "Main Teleop [Use for Worlds]", group = "Teleop")
public class TeleopMain extends OpMode {

    private final double ANALOG_THRESHOLD = 0;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMecanum robot = new HardwareMecanum();

    /* Gamepad values managers */
    private JoystickTransform transform = new JoystickTransform();
    private StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1), stickyGamepad2 = new StickyGamepad(gamepad2);

    /* Robot controller's background manager */
    private BackgroundColorManager background = new BackgroundColorManager();

    /* Handle button positions */
    boolean drivetrainSlowMode, leftStickPressed;

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
        Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
//        robot.drive.setVelocity(v);


//        if (drivetrainSlowMode) {
//            background.setOrangeBackground();
//            robot.drivetrain.tankVectorDrive(leftYInput, rightYInput, slideInput);
//        } else {
//            background.setGreenBackground();
//            robot.drivetrain.tankDriveScaled(leftYInput, rightYInput, slideInput);
//        }

        /**
         * Telemetry
         */
        double[] positions = robot.drivetrain.getPositions();
        telemetry.addData("Encoder counts", "lf: %.2f | rf: %.2f | lb: %.2f | rb: %.2f",
                positions[0],
                positions[1],
                positions[2],
                positions[3]);
    }

    @Override
    public void stop() {
        background.resetBackgroundColor();
    }
}