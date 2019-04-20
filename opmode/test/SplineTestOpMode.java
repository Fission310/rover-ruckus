//package org.firstinspires.ftc.teamcode.opmode.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREVOptimized;
//import org.firstinspires.ftc.teamcode.hardware.mecanum.DriveConstants;
//import org.firstinspires.ftc.teamcode.hardware.mecanum.Drivetrain;
//import org.firstinspires.ftc.teamcode.util.DashboardUtil;
//import org.firstinspires.ftc.teamcode.util.motion.DashboardUtil;
//
///*
// * This is an example of a more complex path to really test the tuning.
// */
//@Autonomous(name="Mecanum Spline Follower")
//public class MecanumSplineFollower extends LinearOpMode {
//
//    @Override
//    public void runOpMode () {
//        Drivetrain drive = new Drivetrain();
//        drive.init(hardwareMap);
//        drive.encoderInit();
//        drive.imuInit(hardwareMap);
//
//        TrajectoryBuilder builder = new TrajectoryBuilder(
//                new Pose2d(0,0,0),
//                DriveConstants.BASE_CONSTRAINTS);
//
//        builder
//                .splineTo(new Pose2d(48, 48, Math.PI / 2), new ConstantInterpolator(0))
//                .splineTo(new Pose2d(0,0,Math.PI), new LinearInterpolator(Math.PI / 2, Math.PI));
//
//        Trajectory trajectory = builder.build();
//
//        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
//                drive,
//                DemoMecanumDrive.TRANSLATIONAL_COEFFICIENTS,
//                DemoMecanumDrive.HEADING_COEFFICIENTS,
//                DemoMecanumDrive.K_V,
//                DemoMecanumDrive.K_A,
//                DemoMecanumDrive.K_STATIC
//        );
//
//        waitForStart();
//
//        follower.followTrajectory(trajectory);
//
//        while (opModeIsActive() && follower.isFollowing()) {
//            follower.update(drive.getPoseEstimate());
//        }
//
//    }
//}
