package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.opmode.Steps;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;

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

    private Steps.State step = Steps.State.HANG_AND_SAMPLE;

    private int ROTATIONS = 0;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Initialize CV
        visionManager.init(hardwareMap, false);
        visionManager.vuforiaLights(true);
        visionManager.start();

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            goldLocation = visionManager.getDoubleMineralLocation();
            telemetry.addData("Status", "Waiting in Init");
            telemetry.addData("Gold location:", goldLocation);
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        while (opModeIsActive() && !isStopRequested() && isStarted()) {
            switch (step) {
                /**
                 * Hang and scan for the gold mineral location.
                 */
                case HANG_AND_SAMPLE:
                    telemetry.addData("Gold Location not found during init. location:", "trying again");
                    //try to find gold location again
                    goldLocation = visionManager.getDoubleMineralLocation();
                    ElapsedTime elapsedTime = new ElapsedTime();
                    while(elapsedTime.seconds() < 2) ;
                    telemetry.addData("Gold Location", goldLocation);
                    telemetry.addData("Test", "Test");
                    step = Steps.State.LAND;
                    break;
                /**
                 * Land and wait for the robot to fully drop and stabilize.
                 */
                case LAND:
                    robot.land();
                    telemetry.addData("Status", "Robot Landed");
                    telemetry.addData("Gold Cube location", goldLocation);
                    telemetry.update();
                    step = Steps.State.IMU_INIT;
                    break;
                /**
                 * IMU Init.
                 */
                case IMU_INIT:
                    robot.imuInit(hardwareMap);
//                    robot.drivetrain.resetDeltaAngle();
//                    robot.drivetrain.imuStartingRot();
                    telemetry.addData("Imu", "Initialized");
                    telemetry.addData("Gold Cube location", goldLocation);
                    telemetry.update();
                    step = Steps.State.TURN_OFF_CV;
                    break;
                /**
                 * Figure out where the gold cube is.
                 */
                case FIND_GOLD_LOCATION:
                    /**
                     * Makes sure that the gold location is found.
                     */
                    //if gold location was not found during init
                    while (goldLocation == TensorFlowManager.TFLocation.NONE && ROTATIONS < 18 && runtime.seconds() > 15) {
                        telemetry.addData("Gold Location not found during init. location:", "trying again");
                        telemetry.update();
                        ROTATIONS += 2;
                        robot.drivetrain.turnPID(2);
                        goldLocation = (goldLocation != TensorFlowManager.TFLocation.NONE) ? goldLocation : visionManager.getDoubleMineralLocation();
                        ElapsedTime elapsedTimes = new ElapsedTime();
                        while(elapsedTimes.seconds() < 1) ;
                    }
                    if (ROTATIONS > 0) { robot.drivetrain.turnPID(-ROTATIONS); }
                    telemetry.addData("Gold Cube location", goldLocation);
                    telemetry.addData("Test", "Test");
                    telemetry.update();
                    step = Steps.State.TURN_OFF_CV;
                    break;
                /**
                 * Turn off CV.
                 */
                case TURN_OFF_CV:
                    visionManager.vuforiaLights(false);
                    telemetry.addData("Status", "Turn off CV");
                    telemetry.update();
                    step = Steps.State.STRAFE_OUT_LANDER;
                    break;
                /**
                 * Slide out of lander.
                 */
                case STRAFE_OUT_LANDER:
                    robot.strafeOutOfLander();
                    telemetry.addData("Status", "Robot strafed out of lander");
                    telemetry.update();
                    step = Steps.State.TURN_90;
                    break;
                /**
                 * Rotate 90 degrees; Robot faces backwards for the marker mechanism.
                 */
                case TURN_90:
                    robot.turn90();
                    telemetry.addData("Robot rotates 90 after strafing out of lander", "");
                    telemetry.update();
                    step = Steps.State.ALIGN_TO_GOLD;
                    break;
                /**
                 * Align the robot to the gold cube to push it in to the depot
                 */
                case ALIGN_TO_GOLD:
                    robot.tfRotateFindGoldLocation(goldLocation);
                    telemetry.addData("Status", "Robot aligned to gold cube");
                    telemetry.update();
                    step = Steps.State.SAMPLE;
                    break;
                /**
                 * Push the gold cube into the depot
                 */
                case SAMPLE:
                    robot.tfDepotSamplePID(goldLocation);
                    telemetry.addData("Status", "Robot Pushed cube into depot");
                    telemetry.update();
                    step = Steps.State.MARKER;
                    break;
                /**
                 * Drop the marker
                 */
                case MARKER:
                    robot.dropMarker();
                    telemetry.addData("Status", "Robot dropped marker");
                    telemetry.update();
                    step = Steps.State.ALIGN_TO_WALL;
                    break;
                /**
                 * Align to wall
                 */
                case ALIGN_TO_WALL:
                    robot.alignToWall(false);
                    telemetry.addData("Status", "Robot align to wall");
                    telemetry.update();
                    step = Steps.State.PARK;
                    break;
                /**
                 * Extend arm and drive up to the crater
                 */
                case PARK:
                    robot.driveToCrater(false);
                    telemetry.addData("Status", "Robot drove to crater");
                    telemetry.update();
                    step = Steps.State.DEFAULT;
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
            visionManager.vuforiaLights(false);
            visionManager.stop();
        }
    }
}
