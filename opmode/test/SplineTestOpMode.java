package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.mecanum.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.mecanum.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.mecanum.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.motion.DashboardUtil;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Mecanum Spline Follower")
public class SplineTestOpMode extends LinearOpMode {
    private HardwareMecanum robot = new HardwareMecanum(this);

    @Override
    public void runOpMode () {
        robot.init(hardwareMap);
        robot.imuInit(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        robot.drivetrain.setPoseEstimate(new Pose2d(13, 13, -45));
        Trajectory trajectory = robot.drivetrain.trajectoryBuilder()
                .forward(48)
                .turn(Math.toRadians(90))
                .forward(48)
                .turn(Math.toRadians(90))
                .forward(48)
                .turn(Math.toRadians(90))
                .forward(48)
                .turn(Math.toRadians(90))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested()) {
            robot.drivetrain.followTrajectory(trajectory);

            while (!isStopRequested() && robot.drivetrain.isFollowingTrajectory()) {
                Pose2d currentPose = robot.drivetrain.getPoseEstimate();
                Pose2d error = robot.drivetrain.getFollowingError();

                TelemetryPacket packet = new TelemetryPacket();
                Canvas fieldOverlay = packet.fieldOverlay();

                packet.put("xError", error.getX());
                packet.put("yError", error.getY());
                packet.put("headingError", error.getHeading());

                fieldOverlay.setStrokeWidth(4);
                fieldOverlay.setStroke("green");
                DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);

                fieldOverlay.setFill("blue");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                dashboard.sendTelemetryPacket(packet);

                robot.drivetrain.update();
            }
        }
    }
}
