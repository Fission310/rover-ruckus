package org.firstinspires.ftc.teamcode.opmode.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.sensors.MRRangeSensor;
import org.firstinspires.ftc.teamcode.util.sensors.REVDistanceSensor;

@TeleOp(name = "Sensor", group = "Teleop")
public class Sensors extends OpMode {

    MRRangeSensor sensorRange = new MRRangeSensor();
    MRRangeSensor sensorRange2 = new MRRangeSensor();
    //REVDistanceSensor distance = new REVDistanceSensor();

    @Override
    public void init() {
        sensorRange.init(hardwareMap, "range");
        sensorRange2.init(hardwareMap, "range2");
        //distance.init(hardwareMap, "dist");

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

        telemetry.addData("Raw Ultrasonic2", sensorRange2.getRangeRawUltrasonic());
        telemetry.addData("Sensor2", sensorRange2.getRangeRawOptical());
        telemetry.addData("Sensor2", sensorRange2.getRangeOpticalCM());
        telemetry.addData("Sensor2", sensorRange2.getRangeDistanceCM());

//        telemetry.addData("distance", distance.getDistanceMM());
//        telemetry.addData("distance", distance.getDistanceCM());
//        telemetry.addData("distance", distance.getDistanceMeter());
//        telemetry.addData("distance", distance.getDistanceInch());


    }

    @Override
    public void stop() { }
}