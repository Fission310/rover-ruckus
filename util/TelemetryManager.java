package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TelemetryManager {
    protected LinearOpMode opMode;

    public TelemetryManager(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void addStatus(String message) {
        opMode.telemetry.addData("Status", message);
        opMode.telemetry.update();
    }
}
