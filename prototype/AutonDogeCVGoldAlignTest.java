package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

@Autonomous(name="Auton Doge Gold Align Test", group="TEST")
//@Disabled
public class AutonDogeCVGoldAlignTest extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Vision Manager*/
    private VisionManager visionManager = new VisionManager();

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize CV
        visionManager.vuforiaGoldAlignInit(hardwareMap);
        visionManager.vuforiaLights(true);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("Detector", "" + visionManager.isGoldAligned());
//            telemetry.addData("Gold pos", "" + visionManager.getGoldPosX());
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.



        // Stop CV
        if (isStopRequested() || !opModeIsActive()) {
            visionManager.vuforiaStop();
            visionManager.vuforiaLights(false);
        }
    }
}
