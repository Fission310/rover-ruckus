package org.firstinspires.ftc.teamcode.prototype;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.sensors.MRRangeSensor;
import org.firstinspires.ftc.teamcode.util.sensors.REVDistanceSensor;

@TeleOp(name = "Gimbal + Sensor Demo", group = "Teleop")
public class DemoGimbalSensor extends OpMode {


    @Override
    public void init() {

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

    }

    @Override
    public void stop() { }
}