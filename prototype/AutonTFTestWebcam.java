package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;

@Autonomous(name="Auton TF Webcam Test", group="TEST")
@Disabled
public class AutonTFTestWebcam extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Vision Manager*/
    private TensorFlowManager visionManager = new TensorFlowManager();

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize CV
        visionManager.init(hardwareMap, true);
        visionManager.vuforiaLights(true);
        visionManager.start();
        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Location", "" + visionManager.getDoubleMineralLocation());
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        /**
         * Figure out where the gold cube is and drive towards it.
         */
        while (runtime.seconds() <= 30 && opModeIsActive()) {
            telemetry.addData("Location", "" + visionManager.getDoubleMineralLocation());
//        location = visionManager.getLocation();
//        telemetry.addData("Gold Cube location after start", location);
            telemetry.update();
        }

//        sleep(30000);


        // Stop .
        if (isStopRequested() || !opModeIsActive()) { visionManager.stop(); }
    }
}
