package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo", group = "Concept")
@Disabled
public class servo extends LinearOpMode {

    private Servo test;

    @Override
    public void runOpMode() {
        test = hardwareMap.servo.get("test");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            if (gamepad2.left_bumper ==  true) {
                test.setPosition(0.00);
            } else if(gamepad2.right_bumper ==  true) {
                test.setPosition(1);
            } else { test.setPosition(.50); }
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
