package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.opmode.Steps;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;

@Autonomous(name="Land Auton: D;S;M;P", group="Slide Depot")
public class LandAutonTest extends LinearOpMode {
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
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        while (opModeIsActive() && !isStopRequested()) {
            switch (step) {
                /**
                 * Hang and scan for the gold mineral location.
                 */
                case HANG_AND_SAMPLE:
                    if (goldLocation != TensorFlowManager.TFLocation.NONE) {
                        telemetry.addData("Gold Location found during init. location:", goldLocation);
                    } else {
                        telemetry.addData("Gold Location not found during init. location:", "trying again");
                        //try to find gold location again
                        goldLocation = visionManager.getDoubleMineralLocation();
                        ElapsedTime elapsedTime = new ElapsedTime();
                        while(elapsedTime.seconds() < 2) ;
                    }
                    telemetry.addData("Gold Location", goldLocation);
                    telemetry.update();
                    step = step.LAND;
                    break;
                /**
                 * Land and wait for the robot to fully drop and stabilize.
                 */
                case LAND:
                    robot.testland();
                    telemetry.addData("Status", "Robot Landed");
                    telemetry.update();
                    step = step.IMU_INIT;
                    break;
                /**
                 * IMU Init.
                 */
                case IMU_INIT:
                    robot.imuInit(hardwareMap);
//                    robot.drivetrain.resetDeltaAngle();
//                    robot.drivetrain.imuStartingRot();
                    telemetry.addData("Imu", "Initialized");
                    telemetry.update();
                    step = step.TURN_OFF_CV;
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
