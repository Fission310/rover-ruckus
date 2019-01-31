package org.firstinspires.ftc.teamcode.opmode.auton;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.opmode.Steps;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

@Autonomous(name="Main TF Depot: D;S;M;P", group="Slide Depot")
public class DepotMainTF extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

    /* Vision Manager*/
    private TensorFlowManager visionManager = new TensorFlowManager();

    /* Gold Location*/
    private TensorFlowManager.TFLocation goldLocation = TensorFlowManager.TFLocation.NONE;

    private Steps.State step = Steps.State.LAND;

    private static int ROTATIONS = 0;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Initialize CV
        visionManager.init(hardwareMap, true);
//        visionManager.vuforiaLights(true);
        visionManager.start();

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
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
                    step = step.STRAFE_OUT_LANDER;
                    break;

                case STRAFE_OUT_LANDER:
                    robot.drivetrain.strafeToPos(.8, 10, 2);
                    robot.turn90();
                    telemetry.addData("Status", "Robot turned 90 degrees");
                    telemetry.update();
                    step = step.FIND_GOLD_LOCATION;
                    break;

                /**
                 * Figure out where the gold cube is.
                 */
                case FIND_GOLD_LOCATION:
                    goldLocation = visionManager.getDoubleMineralLocation();
                    while (runtime.seconds() <= 15 && goldLocation == TensorFlowManager.TFLocation.NONE && ROTATIONS < 14) {
                        goldLocation = (goldLocation != TensorFlowManager.TFLocation.NONE) ? goldLocation : visionManager.getDoubleMineralLocation();
                        ROTATIONS += 2;
                        robot.drivetrain.turnPID(2);
                    }
                    robot.drivetrain.turnPID(-ROTATIONS);
                    telemetry.addData("Gold Cube location after start", goldLocation);
                    telemetry.update();
                    step = step.ALIGN_TO_GOLD;
                    break;

                case ALIGN_TO_GOLD:
                    robot.tfDepotFindGoldLocation(visionManager, goldLocation);
                    telemetry.addData("Status", "Robot driven to gold cube");
                    telemetry.update();
                    step = step.SAMPLE;
                    break;

                /**
                 * Align the robot to the gold cube to push it in to the depot
                 */
                case SAMPLE:
                    robot.tfDepotSamplePID(visionManager, goldLocation);
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
        if (isStopRequested() || !opModeIsActive()) { visionManager.stop(); }
    }
}
