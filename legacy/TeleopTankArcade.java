package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.legacy.hardware.tankdrive.HardwareTank;
import org.firstinspires.ftc.teamcode.util.SoundManager;

import static java.lang.Math.abs;

/**
 * TeleopTankMain is the primary TeleOp OpMode for tank drivetrains. All driver-controlled actions should
 * be defined in this class. This teleop uses one analog sticks to driveArcade the robot
 *
 * Gamepad1 BUTTON MAPPINGS:
 * Left stick x:      N/A
 * Left stick y:      N/A
 * Right stick x:     Turn Robot
 * Right stick y:   Control robot's velocity and direction
 * X:               Un-extends sweeper arm
 * Y:               Extends sweeper arm
 * A:               Un-extends drop arm
 * B:               Extends drop arm
 * Left bumper:     Decelerates robot
 * Right bumper:    Accelerates robot
 * Left trigger:    Special CV program: driveArcade to lander - drives slightly left of the middle of the lander + lifts tape measure
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
@TeleOp(name = "Teleop: Tank", group = "Teleop")
@Disabled
public class TeleopTankArcade extends OpMode {

    private static final double ANALOG_THRESHOLD = 0.10;
    private static final double SLOW_MULTIPLIER = 0.2;
    private static final double FAST_MULTIPLIER = 2;

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareTank robot = new HardwareTank();

    /* Sound manager */
    private SoundManager soundManager = new SoundManager();

    double yInput, xInput, hangingInput;

    private boolean goldFound, silverFound;
    private boolean isX, isY, wasX, wasB = false;    // Gamepad button state variables

    private int silverSoundID, goldSoundID;

    @Override
    public void init() {
        // Retrieve sound ids from raw folder
        goldSoundID = soundManager.getSoundID(hardwareMap, "gold");
        silverSoundID = soundManager.getSoundID(hardwareMap, "silver");

        // Checks if soundId has an id and loads them
        if (goldSoundID != 0)
            goldFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);
        if (silverSoundID != 0)
            silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);
    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see OpMode#init_loop()
     */
    @Override
    public void init_loop() {
        robot.waitForStart();
    }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see OpMode#loop()
     */
    @Override
    public void start() {
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

//        robot.servoArm.armUp();
//        robot.servoArm.sweeperNeutral();

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

        yInput = gamepad1.left_stick_y;
        xInput = gamepad1.left_stick_x;
        hangingInput = -gamepad2.left_stick_y;

        telemetry.addData("Status", "yinput: " + yInput);

        /**
         * Threshold for strafing, makes horizontal strafing easier
         */
        if (abs(gamepad2.left_stick_y) < ANALOG_THRESHOLD) {
            yInput = 0;
        }

        /**
         * Drives the robot based on driver joystick input, check for slow mode
         */
        if (gamepad1.left_bumper) {
            robot.drivetrain.driveArcade(yInput * SLOW_MULTIPLIER, xInput * SLOW_MULTIPLIER);
        } else if (gamepad1.right_bumper ) {
            robot.drivetrain.driveArcade(yInput * FAST_MULTIPLIER, xInput * FAST_MULTIPLIER);
        } else {
            robot.drivetrain.driveArcade(yInput, xInput);
        }

        /**
         * Threshold for hanger, makes hanging easier
         */
        if (abs(gamepad2.left_stick_y) < ANALOG_THRESHOLD) {
            hangingInput = 0;
        }

        /**
         * Controls the hanging mechanism, check for slow mode
         */
        if (gamepad2.left_bumper) {
            robot.hanger.setHangerPower(hangingInput * SLOW_MULTIPLIER);
        } else if (gamepad2.right_bumper ) {
            robot.hanger.setHangerPower(hangingInput * FAST_MULTIPLIER);
        } else {
            robot.hanger.setHangerPower(hangingInput);
        }

        /**
         * Set arms position
         */
        if (gamepad1.a || gamepad2.a) robot.marker.markerLeft();
        else if (gamepad1.b || gamepad2.b) robot.marker.markerNeutral();

        /**
         * Controllers spool
         */
        if (gamepad2.x) robot.hanger.spool(1);
        else if (gamepad2.y) robot.hanger.spool(.5);
        else robot.hanger.spool(0);

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