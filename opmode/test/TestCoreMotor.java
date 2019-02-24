package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "testCoreHex", group = "Teleop")
//@Disabled
public class TestCoreMotor extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;


    @Override
    public void init() {
        leftDrive  = hardwareMap.get(DcMotor.class, "l");
        rightDrive  = hardwareMap.get(DcMotor.class, "r");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see OpMode#init_loop()
     */
    @Override
    public void init_loop() {
//        robot.waitForStart();
        telemetry.addData("Status:", "Waiting to start");
        telemetry.update();
    }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see OpMode#loop()
     */
    @Override
    public void start() {

    }

    /**
     * Runs continuously while the OpMode is active. Defines the driver-controlled actions
     * according to gamepad input.
     * @see OpMode#loop()
     */
    @Override
    public void loop() {
        double leftPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower = -drive ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(leftPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f)", leftPower);
    }

    @Override
    public void stop() { }
}