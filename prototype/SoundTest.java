package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.SoundManager;

@TeleOp(name="Concept: Sound Resources", group="Concept")
@Disabled
public class SoundTest extends LinearOpMode {

    SoundManager soundManager = new SoundManager();

    // Declare OpMode members.
    private boolean goldFound;      // Sound file present flags
    private boolean silverFound;

    private boolean isX = false;    // Gamepad button state variables
    private boolean isB = false;

    private boolean wasX = false;   // Gamepad button history variables
    private boolean WasB = false;


    @Override
    public void runOpMode() {

        // Determine Resource IDs for sounds built into the RC application.
        int silverSoundID = soundManager.getSoundID(hardwareMap, "canttouch");
        int goldSoundID   = soundManager.getSoundID(hardwareMap, "canttouch");

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (goldSoundID != 0)
            goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);

        if (silverSoundID != 0)
            silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);

        // Display sound status
        telemetry.addData("gold resource",   goldFound ?   "Found" : "NOT found\n Add gold.wav to /src/main/res/raw" );
        telemetry.addData("silver resource", silverFound ? "Found" : "Not found\n Add silver.wav to /src/main/res/raw" );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to continue");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Press X, B to play sounds.");
        telemetry.update();
        soundManager.playSound(hardwareMap.appContext, silverSoundID);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData(">", "works.");
            telemetry.update();
            // say Silver each time gamepad X is pressed (This sound is a resource)
            if (silverFound && (isX = gamepad1.x) && !wasX) {
                soundManager.playSound(hardwareMap.appContext, silverSoundID);
                telemetry.addData("Playing", "Resource Silver");
                telemetry.update();
            }

            // say Gold each time gamepad B is pressed  (This sound is a resource)
            if (goldFound && (isB = gamepad1.b) && !WasB) {
                soundManager.playSound(hardwareMap.appContext, goldSoundID);
                telemetry.addData("Playing", "Resource Gold");
                telemetry.update();
            }

            // Save last button states
            wasX = isX;
            WasB = isB;
        }
    }
}
