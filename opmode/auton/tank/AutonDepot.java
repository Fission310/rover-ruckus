package org.firstinspires.ftc.teamcode.opmode.auton.tank;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.tankdrive.HardwareTank;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;
import org.firstinspires.ftc.teamcode.util.SoundManager;

@Autonomous(name="Depot: Drop + Sample + Marker", group="Depot")
@Disabled
public class AutonDepot extends LinearOpMode {

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
        // Initialize CV
        VisionManager visionManager = new VisionManager();
        visionManager.goldAlignInit(hardwareMap);

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

        /**
         * Land and wait for the robot to fully drop and stabilize.
         */
//        robot.land();
//        sleep(2000);

        /**
         * Rotate and driveArcade to the far end of sampling.
         */
        robot.drivetrain.turnPID(-45);
        robot.drivetrain.driveToPos(0.4, FieldConstants.FLOOR_TILE * 1.5, FieldConstants.FLOOR_TILE * 1.5, 4.0);
        robot.drivetrain.turnPID(135);

        /**
         * Start the sampling.
         */
        robot.samplePID(visionManager);

        /**
         * Drive to wall and rotate to marker depot, drop marker and then driveArcade to park.
         */
        robot.drivetrain.driveToPos(0.4, FieldConstants.FLOOR_TILE + 6,FieldConstants.FLOOR_TILE + 6,3);
        robot.drivetrain.turnPID(-135);
        robot.drivetrain.driveToPos(0.7, -FieldConstants.FLOOR_TILE * 3,-FieldConstants.FLOOR_TILE * 3,5);
        robot.drivetrain.driveToPos(1.0, FieldConstants.FLOOR_TILE * 4,FieldConstants.FLOOR_TILE * 4,5);

        // Stop CV
        visionManager.goldAlignStop();
    }
}
