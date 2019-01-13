package org.firstinspires.ftc.teamcode.opmode.auton.slide;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

@Autonomous(name="Main Crater: P22 red outward", group="Slide Depot")
public class AutonCraterP2 extends LinearOpMode {

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
        robot.drivetrain.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.


//        robot.acquirer.acquirerRotationInit();
        telemetry.addData("Status", "Robot Init");
        telemetry.update();

        /**
         * Land and wait for the robot to fully drop and stabilize.
         */

        robot.land();
        telemetry.addData("Status", "Robot Landed");
        telemetry.update();

        /**
         * Straight to Crater
         */
        robot.drivetrain.driveToPos(.5, -FieldConstants.TILE_HYPOTENUSE * 2.5, -FieldConstants.TILE_HYPOTENUSE * 2.5, 6);
        telemetry.addData("Status", "Robot Landed");
        telemetry.update();
        robot.dropMarker();
        robot.marker.markerRight();
        robot.marker.markerLeft();
        robot.marker.markerNeutral();
        robot.marker.markerLeft();
        robot.drivetrain.turn(-45, 5);
        robot.alignToWall();
        robot.driveToCrater();
//

//        robot.drivetrain.driveToPos(.5, -FieldConstants.TILE_HYPOTENUSE /5, -FieldConstants.TILE_HYPOTENUSE /5, 6);








    }
}
