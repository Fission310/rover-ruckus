package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import org.firstinspires.ftc.teamcode.util.VisionManager;

/*
    This autonomous program is used to test CV methods without the availability of a robot
 */
@Autonomous(name="Auton: Hanging + Sampling", group="Auton")
public class CVAutonTest extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
//    private HardwareMain robot = new HardwareMain(this);

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize CV
        VisionManager visionManager = new VisionManager();
        visionManager.goldAlignInit(hardwareMap);

        // Initialize robot
//        robot.init(hardwareMap);

        // Wait until we're told to go
        waitForStart();
        while (opModeIsActive()) {
//            runtime.reset();  // Start counting run time from now.

//        robot.getCubeDetails();
        }
        // Stop CV
        visionManager.vuforiaStop();
        visionManager.goldAlignStop();

    }
}
