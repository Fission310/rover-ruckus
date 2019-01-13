package org.firstinspires.ftc.teamcode.prototype;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.PathTrajectorySegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Depot: Test Path Finding", group="Slide Depot")
@Disabled
public class AutonDepotTest extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

//    public void followPath (Path path) {
//        trajectory = new Trajectory(path);
//    }
//
//    public boolean isFollowingPath () {
//        return !trajectory.isComplete() && currentMode == Mode.FOLLOWING_PATH;
//    }

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        robot.drivetrain.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.BRAKE);

        List<Path> paths = new ArrayList<>();

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        paths.add(new PathBuilder(new Pose2d(0,0,0))
                .splineTo(new Pose2d(50, 24, 0), new ConstantInterpolator(0))
                .splineTo(new Pose2d(90, 36, Math.PI/2), new ConstantInterpolator(0))
                .splineTo(new Pose2d(50, 72, Math.PI), new ConstantInterpolator(0))
                .splineTo(new Pose2d(36, 36, -Math.PI/2), new ConstantInterpolator(0))
                .splineTo(new Pose2d(12, 36, Math.PI/2), new ConstantInterpolator(0))
                .splineTo(new Pose2d(12, 90, Math.PI/2), new ConstantInterpolator(0))
                .build());

//        for (Path path: paths) {
//            robot.drive.followPath(path);
//            while (robot.drive.isFollowingPath()) {
//                if (isStopRequested()) {
//                    robot.drive.stop();
//                    return;
//                }
//            }
//        }

    }
}
