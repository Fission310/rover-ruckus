package org.firstinspires.ftc.teamcode.prototype.PIDPresentation;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.SoundManager;

@Autonomous(name="Rotate 90 With PID", group="PID")
public class Rotate90WithPID extends LinearOpMode {

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

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        /**
         * Can't Touch this song = turn 45 then wait 4 seconds
         */
        robot.drivetrain.turnPID(-90);

        sleep(2000);

        robot.drivetrain.driveToPos(.3,5.0,5.0,4);

        sleep(2000);

        robot.drivetrain.driveToPos(.3,-5.0,-5.0,4);

        sleep(2000);

        /**
         * Turn -45 degrees
         */
        robot.drivetrain.turnPID(90);
    }
}
