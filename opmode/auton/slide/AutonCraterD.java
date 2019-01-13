package org.firstinspires.ftc.teamcode.opmode.auton.slide;

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

@Autonomous(name="Main Crater : D", group="Slide Depot")
public class AutonCraterD extends LinearOpMode {

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
                    robot.land();
                    telemetry.addData("Status", "Robot Landed");
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
