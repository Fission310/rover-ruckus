package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

public class MotionProfiler {
    private Path path;
    private MotionProfile axialProfile;

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0.0, 0.0, 0.0, 0.0), // start state
            new MotionState(10.0, 0.0, 0.0, 0.0), // goal state
            5, // max vel
            5 // max accel
    );

}
