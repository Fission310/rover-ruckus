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

@Autonomous(name="Auton TF Test", group="TEST")
public class AutonTFTest extends LinearOpMode {

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
        visionManager.init(hardwareMap);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            visionManager.start();
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        telemetry.addData("TF location", visionManager.getLocation());


        /**
         * Land and wait for the robot to fully drop and stabilize.
         */
        sleep(30000);


        // Stop CV
        if (isStopRequested() || !opModeIsActive()) { visionManager.stop(); }
    }
}
