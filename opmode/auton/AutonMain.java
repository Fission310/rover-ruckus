package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

@Autonomous(name="Auton: Hanging + Sampling + Marker + Crater", group="Auton")
public class AutonMain extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareMain robot = new HardwareMain(this);

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

        // Wait until we're told to go
        waitForStart();
        runtime.reset();  // Start counting run time from now.

//        robot.getCubeDetails();
        sleep(1000);

        // Stop CV
        visionManager.vuforiaStop();
        visionManager.goldAlignStop();

    }
}
