package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;

@Autonomous(name="PID Test 1/26/19", group="Test")
public class AutonPIDTest extends LinearOpMode {

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
//                    robot.drivetrain.driveToPos(.5, -FieldConstants.FLOOR_TILE, -FieldConstants.FLOOR_TILE, 5.00);
                    step++;
                    break;
                case 1:
                    telemetry.addData("Step 0", "Robot Drive one floor tile negative speed");
                    telemetry.update();
                    step++;
                    break;

                case 2:
                    sleep(2000);
                    telemetry.addData("Step 2", "Robot PID turn");
                    telemetry.update();
                    robot.drivetrain.turnPID(90);
                    step++;
                    break;
                case 3:
                    sleep(2000);
                    step++;
                    break;

                case 4:
                    telemetry.addData("Step 4", "Robot strafe");
                    telemetry.update();
//                    robot.drivetrain.strafeToPos(.8, FieldConstants.FLOOR_TILE, 5);
                    step++;
                    break;

                case 5:
                    sleep(3000);
                    robot.drivetrain.turnPID(90);
                    sleep(1000);
                    robot.drivetrain.turnPID(90);
                    sleep(1000);
                    robot.drivetrain.turnPID(90);
                    sleep(1000);
                    step++;
                    break;
                case 6:
                    telemetry.addData("Step 5", "Robot strafe pid");
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
        }
}
