package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.TelemetryManager;

@Autonomous(name="Telemetry Test", group="Slide Depot")
public class TelemetryTest extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();
    private TelemetryManager telemetryManager = new TelemetryManager(this);

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        while (opModeIsActive()) {
//            if (runtime.seconds() < 5) {
            telemetryManager.addStatus("Test");
//                telemetry.addData("Status", "1");
//                telemetry.update();
//            } else if (runtime.seconds() < 10) {
//                if (runtime.seconds() ==  15) {
//                    telemetry.addData("Status", "1");
//                } if (runtime.seconds() ==  18) {
//                    telemetry.addData("Status", "1");
//                }
//            }
        }

        // Stop CV
        if (isStopRequested() || !opModeIsActive()) { }
    }
}
