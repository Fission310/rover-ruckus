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

@Autonomous(name="Test PID and IMU turn and drive straight 1/14/19", group="Test")
public class AutonTestDrive extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

    private int step = 0;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        robot.imuInit(hardwareMap);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        while (opModeIsActive()) {
            switch (step) {
                case 0:
                    telemetry.addData("Step 0", "Robot Drive one floor tile");
                    telemetry.update();
                    robot.drivetrain.driveToPos(.5, FieldConstants.FLOOR_TILE, FieldConstants.FLOOR_TILE, 5.00);
                    step++;
                    break;

                case 1:
                    telemetry.addData("Step 1", "Robot PID turn");
                    telemetry.update();
                    robot.drivetrain.turnPID(90);
                    step++;
                    break;

                case 2:
                    telemetry.addData("Step 2", "Robot strafe");
                    telemetry.update();
                    robot.drivetrain.strafeToPos(.8, FieldConstants.FLOOR_TILE, 9);
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
        }
}
