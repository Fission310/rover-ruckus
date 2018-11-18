package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareTank;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;
import org.firstinspires.ftc.teamcode.util.SoundManager;
import org.firstinspires.ftc.teamcode.hardware.Constants;

@Autonomous(name="Auton: Drive straight Crater", group="Auton")
public class AutonCrater extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareTank robot = new HardwareTank(this);

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();

        // Wait until we're told to go
        waitForStart();
        runtime.reset();  // Start counting run time from now.

        robot.drivetrain.driveToPos(1, 140,140,30);

    }
}
