package org.firstinspires.ftc.teamcode.opmode.auton.slide;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

@Autonomous(name="Depot: Drop + Sample + Marker", group="Slide Depot")
public class AutonDepot extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

    /* Vision Manager*/
    private VisionManager visionManager = new VisionManager();

    /* Gold Location*/
    private SamplingOrderDetector.GoldLocation goldLocation;

    private int step = 0;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Initialize CV
        visionManager.samplingInit(hardwareMap);
        goldLocation = visionManager.getGoldLocation();
        telemetry.addData("Gold Cube location before start", goldLocation);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            if (goldLocation == goldLocation.UNKNOWN) {
                goldLocation = visionManager.getGoldLocation();
            }
            telemetry.addData("Location", "gold cube location:" + goldLocation);
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        switch (step){
            /**
             * Land and wait for the robot to fully drop and stabilize.
             */
            case 0:
                robot.land();
//                if (goldLocation == goldLocation.UNKNOWN) step = 1;
                step++;
                break;

            /**
             * Figure out where the gold cube is.
             */
            case 1:
                goldLocation = (goldLocation != goldLocation.UNKNOWN) ? goldLocation : visionManager.getGoldLocation();
                telemetry.addData("Gold Cube location after start", goldLocation);
//                while (location == goldLocation.NONE){
//                    robot.drivetrain.driveToPos(.3,2,2,2);
//                    location = visionManager.getGoldLocation();
//                }
                robot.findGoldLocation(visionManager, goldLocation);
                step++;
                break;

            /**
             * Align the robot to the gold cube to push it in to the depot
             */
            case 2:
                robot.samplePID(visionManager, goldLocation);
                step++;
                break;

            /**
             * Drop the marker
             */
            case 3:
//        robot.dropMarker()
                step++;
                break;

            /**
             * Align to wall
             */
            case 4:
//        robot.alignToWall()
                step++;
                break;

            /**
             * Extend arm and drive up to the crater
             */
            case 5:
//        robot.driveToCrater()
                step++;
                break;

            default: {
                robot.drivetrain.drive(0, 0);
            }
            break;
        }

        // Stop CV
        if (isStopRequested() || !opModeIsActive()) { visionManager.samplingStop(); }
    }
}
