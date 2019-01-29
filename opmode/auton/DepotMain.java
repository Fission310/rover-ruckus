package org.firstinspires.ftc.teamcode.opmode.auton;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.opmode.Steps;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

@Autonomous(name="Main Depot: D;S;M;P", group="Slide Depot")
public class DepotMain extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

    /* Vision Manager*/
    private VisionManager visionManager = new VisionManager();

    /* Gold Location*/
    private SamplingOrderDetector.GoldLocation goldLocation;

    private Steps.State step = Steps.State.LAND;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Initialize CV
//        visionManager.samplingInit(hardwareMap);
//        visionManager.vuforiaInit(hardwareMap);
        visionManager.vuforiaSampleInit(hardwareMap);
//        goldLocation = visionManager.getGoldLocation();
        visionManager.vuforiaLights(true);
        telemetry.addData("Gold Cube location before start", goldLocation);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Location", "gold cube location:" + goldLocation);
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        while (opModeIsActive()) {
            switch (step) {
                /**
                 * Land and wait for the robot to fully drop and stabilize.
                 */
                case LAND:
                    robot.land();
                    telemetry.addData("Status", "Robot Landed");
                    telemetry.update();
                    step = step.IMU_INIT;
                    break;

                case IMU_INIT:
                    robot.imuInit(hardwareMap);
                    telemetry.addData("Imu", "Initialized");
                    telemetry.update();
                    step = step.FIND_GOLD_LOCATION;
                    break;

                /**
                 * Figure out where the gold cube is.
                 */
                case FIND_GOLD_LOCATION:
                    goldLocation = (goldLocation != goldLocation.UNKNOWN) ? goldLocation : visionManager.getGoldLocation();
                    telemetry.addData("Gold Cube location after start", goldLocation);
                    telemetry.update();
                    step = step.STRAFE_OUT_LANDER;
                    break;

                case STRAFE_OUT_LANDER:
                    robot.drivetrain.strafeToPos(.8, -8, 2);
                    robot.turn90();
                    telemetry.addData("Status", "Robot turned 90 degrees");
                    telemetry.update();
                    step = step.ALIGN_TO_GOLD;
                    break;

                case ALIGN_TO_GOLD:
                    robot.depotFindGoldLocation(visionManager, goldLocation);
                    telemetry.addData("Status", "Robot driven to gold cube");
                    telemetry.update();
                    step = step.SAMPLE;
                    break;

                /**
                 * Align the robot to the gold cube to push it in to the depot
                 */
                case SAMPLE:
                    robot.depotSamplePID(visionManager, goldLocation);
                    telemetry.addData("Status", "Robot Pushed cube into depot");
                    telemetry.update();
                    step = step.MARKER;
                    break;

                /**
                 * Drop the marker
                 */
                case MARKER:
                    robot.dropMarker();
                    telemetry.addData("Status", "Robot dropped marker");
                    telemetry.update();
                    step = step.ALIGN_TO_WALL;
                    break;

                /**
                 * Align to wall
                 */
                case ALIGN_TO_WALL:
                    robot.alignToWall();
                    telemetry.addData("Status", "Robot align to wall");
                    telemetry.update();
                    step = step.PARK;
                    break;

                /**
                 * Extend arm and drive up to the crater
                 */
                case PARK:
                    robot.driveToCrater();
                    telemetry.addData("Status", "Robot drove to crater");
                    telemetry.update();
                    step = step.DEFAULT;
                    break;

                default: {
                    robot.drivetrain.drive(0, 0);
                    telemetry.addData("Status", "Robot default");
                    telemetry.update();
                }
                break;
            }
        }

        // Stop CV
        if (isStopRequested() || !opModeIsActive()) {
            visionManager.vuforiaStop();
            visionManager.samplingStop();
        }
    }
}
