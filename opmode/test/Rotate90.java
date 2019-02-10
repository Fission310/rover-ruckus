package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;

@Autonomous(name="Rotate90 Test", group="Test")
public class Rotate90 extends LinearOpMode {

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
        robot.drivetrain.resetDeltaAngle();

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
                    telemetry.addData("Rotate 90", "Robot rotate 90");
                    telemetry.update();
                    robot.drivetrain.turnPID(88);
                    step++;
                    break;
                case 1:
                    sleep(4000);
                    telemetry.addData("Rotate 9 0", "Robot rotate 90");
                    telemetry.update();
                    robot.drivetrain.turnPID(-(int)robot.drivetrain.singleImu.getDeltaStartingAngle());
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
