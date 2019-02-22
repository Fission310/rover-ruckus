package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.util.ColorConstants;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;

@Autonomous(name="Color Change", group="Test")
//@Disabled
public class ColorChange extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();
    View relativeLayout;

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

//        try {
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.BLACK);
//                }
//            });
//        } finally {
//            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
//            // as pure white, but it's too much work to dig out what actually was used, and this is good
//            // enough to at least make the screen reasonable again.
//            // Set the panel back to the default color
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.RED);
//                }
//            });
//        }

//        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLUE);
                }
            });
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        while (opModeIsActive() && !isStopRequested() && isStarted()) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, ColorConstants.greenHSVColor));
                }
            });
        }

        if (isStopRequested() || !opModeIsActive()) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, ColorConstants.orangeHSVColor));
                }
            });
        }
    }
}
