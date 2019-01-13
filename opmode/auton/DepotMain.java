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

        while (opModeIsActive()) {
            switch (step) {
                /**
                 * Initiate the robot.
                 */
                case 0:
//                    robot.acquirer.acquirerRotationInit();
                    telemetry.addData("Status", "Robot Init");
                    telemetry.update();
                    step++;
                    break;
                /**
                 * Land and wait for the robot to fully drop and stabilize.
                 */
                case 1:
//                    robot.land();
                    telemetry.addData("Status", "Robot Landed");
                    telemetry.update();
                    step++;
                    break;

                /**
                 * Figure out where the gold cube is.
                 */
                case 2:
                    goldLocation = (goldLocation != goldLocation.UNKNOWN) ? goldLocation : visionManager.getGoldLocation();
                    telemetry.addData("Gold Cube location after start", goldLocation);
                    telemetry.update();
                    step++;
                    break;

                case 3:
                    robot.drivetrain.strafeToPos(.4, -8, 2);
                    robot.turn90();
                    telemetry.addData("Status", "Robot turned 90 degrees");
                    telemetry.update();
                    step++;
                    break;

                case 4:
                    robot.findGoldLocation(visionManager, goldLocation);
                    telemetry.addData("Status", "Robot driven to gold cube");
                    telemetry.update();
                    step++;
                    break;

                /**
                 * Align the robot to the gold cube to push it in to the depot
                 */
                case 5:
                    robot.samplePID(visionManager, goldLocation);
                    telemetry.addData("Status", "Robot Pushed cube into depot");
                    telemetry.update();
                    step++;
                    break;

                /**
                 * Drop the marker
                 */
                case 6:
                    robot.dropMarker();
                    telemetry.addData("Status", "Robot dropped marker");
                    telemetry.update();
                    step++;
                    break;

                /**
                 * Align to wall
                 */
                case 7:
                    robot.alignToWall();
                    telemetry.addData("Status", "Robot align to wall");
                    telemetry.update();
                    step++;
                    break;

                /**
                 * Extend arm and drive up to the crater
                 */
                case 8:
                    robot.driveToCrater();
                    telemetry.addData("Status", "Robot drove to crater");
                    telemetry.update();
                    step++;
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
        if (isStopRequested() || !opModeIsActive()) { visionManager.samplingStop(); }
    }
}
