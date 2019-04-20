package org.firstinspires.ftc.teamcode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.mecanum.HardwareMecanum;
import org.firstinspires.ftc.teamcode.util.signals.BackgroundColorManager;

import static java.lang.Math.abs;

/**
 * TeleopMecanumArcade is the primary TeleOp OpMode for mecanum drivetrains. All driver-controlled actions should
 * be defined in this class.
 */
@TeleOp(name = "Encoder", group = "Teleop")
public class MA3EncodersTest extends OpMode {

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private AnalogSensor encoderInput;
    private AnalogOutput encoderOutput;
    private AnalogSensor encoder;

    @Override
    public void init() {
        encoderInput = hardwareMap.get(AnalogSensor.class, "encoder");
        encoderOutput = hardwareMap.get(AnalogOutput.class, "encoderOutput");
    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see OpMode#init_loop()
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status:", "Waiting to start");
        telemetry.update();
    }

    /**
     * Runs once when the OpMode starts. Starts the OpMode's runtime counter.
     * @see OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * Runs continuously while the OpMode is active. Defines the driver-controlled actions
     * according to gamepad input.
     * @see OpMode#loop()
     */
    @Override
    public void loop() {
        // Adds runtime data to telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Status", "readRawVoltage: " + encoderInput.readRawVoltage());
    }

    @Override
    public void stop() { }
}