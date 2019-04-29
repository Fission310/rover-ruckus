package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.mecanum.HardwareMecanum;
import org.firstinspires.ftc.teamcode.opmode.Steps;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;

@Autonomous(name="Just land", group="Slide Depot")
public class DepotMain2 extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareMecanum robot = new HardwareMecanum(this);

    /* Vision Manager*/
    private TensorFlowManager visionManager = new TensorFlowManager();

    /* Gold Location*/
    private String goldLocation = "CENTER";

    private Steps.State step = Steps.State.LAND;

    private int ROTATIONS = 0;
    private boolean didFindGold = true;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.imuInit(hardwareMap);

        // Initialize CV
        visionManager.init(hardwareMap, false);
        visionManager.vuforiaLights(true);
        visionManager.start();
        AutoTransitioner.transitionOnStop(this, "Linear Teleop [Use for World Champs]");

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            goldLocation = visionManager.getTensorFlow();
            telemetry.addData("Status", "Waiting in Init");
            telemetry.addData("Gyro Is Calibrated", robot.imuCalibrated());
            telemetry.addData("Gold location", goldLocation);
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
                    step = Steps.State.HANG_AND_SAMPLE;
                    break;
                /**
                 * Hang and scan for the gold mineral location.
                 */
                case HANG_AND_SAMPLE:
//                    robot.gimbal.setLandedSamplingPos();
                    didFindGold = false;
                    telemetry.addData("Gold Location", goldLocation);
                    telemetry.update();
                    step = Steps.State.IMU_INIT;
                    break;
                /**
                 * IMU Init.
                 */
                case IMU_INIT:
//                    robot.drivetrain.singleImu.resetAngle();
//                    ElapsedTime elapsedTime = new ElapsedTime();
//                    while(elapsedTime.milliseconds() < .200) ;
                    telemetry.addData("Imu", "Initialized");
                    telemetry.update();
                    step = Steps.State.TURN_OFF_CV;
                    break;
                /**
                 * NOT USED Figure out where the gold cube is.
                 */
                case FIND_GOLD_LOCATION:
                    /**
                     * Makes sure that the gold location is found.
                     */
                    //while (goldLocation == TensorFlowManager.TFLocation.NONE && ROTATIONS < 18 && runtime.seconds() > 15) {
                    //ROTATIONS += 2;
//                    if (goldLocation == TensorFlowManager.TFLocation.NONE) {
//                        robot.drivetrain.turnPID(2);
//                        goldLocation = (goldLocation != TensorFlowManager.TFLocation.NONE) ? goldLocation : visionManager.getDoubleMineralLocation();
//                        ElapsedTime elapsedTime = new ElapsedTime();
//                        while(elapsedTime.seconds() < 1) ;
//                    }
//                    if (ROTATIONS > 0) { robot.drivetrain.turnPID(-ROTATIONS); }
                    telemetry.addData("Gold Cube location", goldLocation);
                    telemetry.update();
                    step = Steps.State.STRAFE_OUT_LANDER;
                    break;
                /**
                 * Turn off CV.
                 */
                case TURN_OFF_CV:
//                    visionManager.vuforiaLights(false);
                    telemetry.addData("Status", "Turn off CV");
                    telemetry.update();
                    step = Steps.State.STRAFE_OUT_LANDER;
                    break;
                /**
                 * Slide out of lander.
                 */
                case STRAFE_OUT_LANDER:
                    robot.strafeOutOfLander();
                    telemetry.addData("Status", "Robot strafed");
                    telemetry.update();
                    step = Steps.State.DEFAULT;
                    break;
                /**
                 * Rotate 90 degrees; Robot faces forwards for the marker mechanism.
                 */
                case TURN_90:
                    robot.turnToSample(goldLocation);
                    telemetry.addData("Robot rotates 90", "");
                    telemetry.update();
                    step = Steps.State.ALIGN_TO_GOLD;
                    break;
                /**
                 * Align the robot to the gold cube to push it in to the depot
                 */
                case ALIGN_TO_GOLD:
//                    robot.tfRotateFindGoldLocation(goldLocation);
                    telemetry.addData("Status", "Robot aligned to gold cube");
                    telemetry.update();
                    step = Steps.State.SAMPLE;
                    break;
                /**
                 * Push the gold cube into the crater
                 */
                case SAMPLE:
                    robot.tfDepotSamplePID(goldLocation);
                    telemetry.addData("Status", "Robot Pushed cube into depot");
                    telemetry.update();
//                    step = Steps.State.MARKER;
                    step = Steps.State.ALIGN_TO_WALL;
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
                    robot.alignToWall(true);
                    telemetry.addData("Status", "Robot align to wall");
                    telemetry.update();
                    step = Steps.State.PARK;
                    break;
                /**
                 * Extend arm and drive up to the crater
                 */
                case PARK:
                    robot.driveToCrater(true);
                    telemetry.addData("Status", "Robot drove to crater");
                    telemetry.update();
                    step = Steps.State.DEFAULT;
                    break;

                default: {
                    robot.drivetrain.setMotorPowers(0, 0, 0, 0);
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
