package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareTank;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;
import org.firstinspires.ftc.teamcode.util.SoundManager;
import org.firstinspires.ftc.teamcode.hardware.Constants;

@Autonomous(name="Auton: Depot Everything", group="Auton")
public class AutonMain extends LinearOpMode {
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
        if (barginID != 0)
            bargin = SoundPlayer.getInstance().preload(hardwareMap.appContext, barginID);
        if (cantTouchID != 0)
            cantTouch = SoundPlayer.getInstance().preload(hardwareMap.appContext, cantTouchID);
        if (einsteinID != 0)
            einstein = SoundPlayer.getInstance().preload(hardwareMap.appContext, einsteinID);
        if (groundID != 0)
            ground = SoundPlayer.getInstance().preload(hardwareMap.appContext, groundID);

        // Wait until we're told to go
        waitForStart();
        runtime.reset();  // Start counting run time from now.
//        soundManager.playSound(hardwareMap.appContext, einsteinID);
        robot.land();
        sleep(2000);

        robot.drivetrain.turn(15.0, 3.0);
        robot.drivetrain.driveToPos(0.3, -4, -4, 4.0);
//        soundManager.stopSound();
//        soundManager.playSound(hardwareMap.appContext, cantTouchID);

        robot.drivetrain.turn(-105.0, 5.0);
        robot.sample(visionManager);
        robot.drivetrain.driveToPos(0.3, 12,12,3);
        robot.drivetrain.turn(105.0, 3.0);
//        soundManager.stopSound();
//        soundManager.playSound(hardwareMap.appContext, barginID);
        robot.drivetrain.driveToPos(0.5, 24,24,3);
        robot.drivetrain.driveToPos(0.5, -50,-50,5);

        // Stop CV
        visionManager.goldAlignStop();
    }
}
