package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;

@Autonomous(name="Lift Test", group="Test")
//@Disabled
public class LiftTest extends LinearOpMode {

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
                    robot.lift.liftToPos(.3, 52);
                    sleep(3000);
                    robot.lift.liftToPos(.3, -52);
                    sleep(3000);
                    robot.lift.liftToPos(.3, 40);
                    telemetry.addData("Step 0", "Robot Drive one floor tile");
                    telemetry.update();
                    step++;
                    break;
                case 1:
//                    sleep(500);\
//                    int distance = (int)(6 * Constants.INCHES_PER_TICK_30);
//                    robot.drivetrain.curveTurn(220,.3,1/4, true);
                    telemetry.addData("Step 0", "Robot Drive one floor tile negative speed");
                    telemetry.update();
                    step++;
                    break;

                default: {
                    robot.drivetrain.driveToPos(0, 0, 5);
                    telemetry.addData("Status", "Robot default");
                    telemetry.update();
                }
                break;
            }
        }
        if (isStopRequested() || !opModeIsActive()) { }
    }
}
