package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.HardwareTank;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;
import org.firstinspires.ftc.teamcode.util.SoundManager;
import org.firstinspires.ftc.teamcode.hardware.Constants;

@Autonomous(name="Tank: TEST STRAIGHT DRIVE", group="Test")
public class AutonTankStraightTest extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareTank robot = new HardwareTank(this);

    /* Sound Manager */
    private SoundManager soundManager = new SoundManager();

    private boolean bargin, cantTouch, einstein, ground;
    private int barginID, cantTouchID, einsteinID, groundID;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Retrieve sound ids from raw folder
        barginID = soundManager.getSoundID(hardwareMap, "bargin");
        cantTouchID = soundManager.getSoundID(hardwareMap, "canttouch");
        einsteinID = soundManager.getSoundID(hardwareMap, "einstein");
        groundID = soundManager.getSoundID(hardwareMap, "ground");

        // Checks if soundId has an id and loads them
        if (barginID != 0) bargin = SoundPlayer.getInstance().preload(hardwareMap.appContext, barginID);
        if (cantTouchID != 0) cantTouch = SoundPlayer.getInstance().preload(hardwareMap.appContext, cantTouchID);
        if (einsteinID != 0) einstein = SoundPlayer.getInstance().preload(hardwareMap.appContext, einsteinID);
        if (groundID != 0) ground = SoundPlayer.getInstance().preload(hardwareMap.appContext, groundID);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        soundManager.playSound(hardwareMap.appContext, einsteinID);

        sleep(2000);
        soundManager.stopSound();

        soundManager.playSound(hardwareMap.appContext, cantTouchID);
        robot.drivetrain.turn(-45, 4.0);

        sleep(4000);
        soundManager.stopSound();

        soundManager.playSound(hardwareMap.appContext, groundID);
        robot.drivetrain.driveToPoss(0.4, FieldConstants.FLOOR_TILE * 2, FieldConstants.FLOOR_TILE * 2, 4.0);

        sleep(4000);
        soundManager.stopSound();

        soundManager.playSound(hardwareMap.appContext, barginID);
        robot.drivetrain.driveToPoss(0.4, -FieldConstants.FLOOR_TILE * 2, -FieldConstants.FLOOR_TILE * 2, 4.0);

        sleep(4000);
        soundManager.stopSound();

        robot.drivetrain.turn(45, 4.0);
    }
}
