package org.firstinspires.ftc.teamcode.opmode.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.sensors.MRRangeSensor;
import org.firstinspires.ftc.teamcode.util.sensors.REVDistanceSensor;

import static java.lang.Math.abs;

@TeleOp(name = "Sensor", group = "Teleop")
public class Sensors extends OpMode {

    MRRangeSensor sensorRange = new MRRangeSensor();
    REVDistanceSensor distance = new REVDistanceSensor();

    @Override
    public void init() {
        sensorRange.init(hardwareMap, "range");
        distance.init(hardwareMap, "dist");

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
        telemetry.addData("Raw Ultrasonic", sensorRange.getRangeRawUltrasonic());
        telemetry.addData("Sensor", sensorRange.getRangeRawOptical());
        telemetry.addData("Sensor", sensorRange.getRangeOpticalCM());
        telemetry.addData("Sensor", sensorRange.getRangeDistanceCM());

        telemetry.addData("distance", distance.getDistanceMM());
        telemetry.addData("distance", distance.getDistanceCM());
        telemetry.addData("distance", distance.getDistanceMeter());
        telemetry.addData("distance", distance.getDistanceInch());


    }

    @Override
    public void stop() { }
}