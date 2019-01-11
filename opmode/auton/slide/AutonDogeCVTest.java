package org.firstinspires.ftc.teamcode.opmode.auton.slide;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;
import org.firstinspires.ftc.teamcode.util.SoundManager;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

@Autonomous(name="Auton Doge Test", group="TEST")
//@Disabled
public class AutonDogeCVTest extends LinearOpMode {

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
        visionManager.goldAlignInit(hardwareMap);
        visionManager.vuforiaInit(hardwareMap);
        visionManager.vuforiaLights(true);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Detector", "" + visionManager.isGoldAligned());
            telemetry.addData("Gold pos", "" + visionManager.getGoldPosX());
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        /**
         * Figure out where the gold cube is and drive towards it.
         */
        telemetry.addData("Detector", "" + visionManager.isGoldAligned());
        telemetry.addData("Gold pos", "" + visionManager.getGoldPosX());
//        location = visionManager.getLocation();
//        telemetry.addData("Gold Cube location after start", location);
        telemetry.update();

//        sleep(30000);


        // Stop CV
        if (isStopRequested() || !opModeIsActive()) {
            visionManager.goldAlignStop();
            visionManager.vuforiaStop();
            visionManager.vuforiaLights(false);
        }
    }
}
