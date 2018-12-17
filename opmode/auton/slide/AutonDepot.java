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

@Autonomous(name="Depot: Drop + Sample + Marker", group="Slide Depot")
public class AutonDepot extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

    /* Vision Manager*/
    private TensorFlowManager visionManager = new TensorFlowManager();

    /* Holds gold cube location*/
    private TensorFlowManager.TFLocation location;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Initialize CV
        visionManager.init(hardwareMap);
        visionManager.start();
        telemetry.addData("TF location", visionManager.getLocation());

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        /**
         * Land and wait for the robot to fully drop and stabilize.
         */
//        robot.land();

        /**
         * Figure out where the gold cube is and drive towards it.
         */
        location = visionManager.getLocation();
//        while (location == TensorFlowManager.TFLocation.NONE){
//            robot.drivetrain.driveToPos(.3,2,2,2);
//            location = visionManager.getLocation();
//        }
//        robot.findGoldLocation();

        /**
         * Align the robot to the gold cube to push it in to the depot
         */
//        robot.samplePID()

        /**
         * Drop the marker
         */
//        robot.dropMarker()

        /**
         * Align to wall
         */
//        robot.alignToWall()

        /**
         * Extend arm and drive up to the crater
         */
//        robot.driveToCrater()


        // Stop CV
        if (isStopRequested() || !opModeIsActive()) { visionManager.stop(); }
    }
}
