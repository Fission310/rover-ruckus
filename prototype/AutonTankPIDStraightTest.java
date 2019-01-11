package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.SoundManager;

@Autonomous(name="Slide: TEST PID STRAIGHT AND ROTATE ", group="Test")
@Disabled
public class AutonTankPIDStraightTest extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        robot.drivetrain.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        robot.drivetrain.turnPID(-90);

        sleep(3000);
//
//        robot.drivetrain.driveToPos(0.3, FieldConstants.FLOOR_TILE * 2, FieldConstants.FLOOR_TILE * 2, 4.0);
//
//        sleep(3000);
//
//        robot.drivetrain.driveToPos(0.3, -FieldConstants.FLOOR_TILE * 2, -FieldConstants.FLOOR_TILE * 2, 4.0);
//
//        sleep(3000);
//
        robot.drivetrain.turnPID(90);
//
        sleep(3000);
//
        robot.drivetrain.turnPID(-90);
//        robot.drivetrain.driveToPos(0.3, FieldConstants.FLOOR_TILE * 2, FieldConstants.FLOOR_TILE * 2, 4.0);
//        robot.drivetrain.driveToPos(0.3, -FieldConstants.FLOOR_TILE * 2, -FieldConstants.FLOOR_TILE * 2, 4.0);
        robot.drivetrain.turnPID(90);

        robot.drivetrain.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
