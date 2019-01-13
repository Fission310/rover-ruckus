package org.firstinspires.ftc.teamcode.opmode.auton.slide;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

@Autonomous(name="Main Depot: S;M;Pyy", group="Slide Depot")
public class AutonDepotSMP extends LinearOpMode {

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
        robot.drivetrain.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize CV
        visionManager.vuforiaInit(hardwareMap);
        visionManager.samplingInit(hardwareMap);
        goldLocation = visionManager.getGoldLocation();
        visionManager.vuforiaLights(true);
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

//                    robot.acquirer.acquirerRotationInit();
                    telemetry.addData("Status", "Robot Init");
                    telemetry.update();
                /**
                 * Land and wait for the robot to fully drop and stabilize.
                 */
                    //robot.land();
                    telemetry.addData("Status", "Robot Landed");
                    telemetry.update();

                /**
                 * Figure out where the gold cube is.
                 */
                    goldLocation = (goldLocation != goldLocation.UNKNOWN) ? goldLocation : visionManager.getGoldLocation();
                    telemetry.addData("Gold Cube location after start", goldLocation);
                    telemetry.update();
                    robot.drivetrain.strafeToPos(.4, -8, 2);
                    telemetry.addData("Status", "Robot turned 90 degrees");
                    telemetry.update();

                    robot.turn90();
                    sleep(2000);
                    robot.findGoldLocation(visionManager, goldLocation);
                    telemetry.addData("Status", "Robot driven to gold cube");
                    telemetry.update();

                /**
                 * Align the robot to the gold cube to push it in to the depot
                 */
                    robot.samplePID(visionManager, goldLocation);
                    telemetry.addData("Status", "Robot Pushed cube into depot");
                    telemetry.update();
                /**
                 * Drop the marker
                 */
                    robot.dropMarker();
                    telemetry.addData("Status", "Robot dropped marker");
                    telemetry.update();

                /**
                 * Align to wall
                 */
                    robot.alignToWall();
                    telemetry.addData("Status", "Robot align to wall");
                    telemetry.update();

                /**
                 * Extend arm and drive up to the crater
                 */
                    robot.driveToCrater();
                    telemetry.addData("Status", "Robot drove to crater");
                    telemetry.update();


        // Stop CV
        if (isStopRequested() || !opModeIsActive()) { visionManager.samplingStop(); }
    }
}
