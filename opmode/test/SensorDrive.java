package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.sensors.MRRangeSensor;
import org.firstinspires.ftc.teamcode.util.sensors.REVDistanceSensor;

@Autonomous(name="SesnorDrive", group="Slide Depot")
public class SensorDrive extends LinearOpMode {
    /* Private OpMode members */
    private ElapsedTime     runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

    MRRangeSensor sensorRange = new MRRangeSensor();
    REVDistanceSensor distance = new REVDistanceSensor();

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
        sensorRange.init(hardwareMap, "range");
        distance.init(hardwareMap, "dist");
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        robot.imuInit(hardwareMap);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        while (opModeIsActive() && !isStopRequested()) {
            if (sensorRange.getRangeRawUltrasonic() > 6) {
                robot.drivetrain.drive(-.4,0);
            } else {
                robot.drivetrain.turnPID(90);
            }

            telemetry.addData("Raw Ultrasonic", sensorRange.getRangeRawUltrasonic());
            telemetry.addData("Sensor", sensorRange.getRangeRawOptical());
            telemetry.addData("Sensor", sensorRange.getRangeOpticalCM());
            telemetry.addData("Sensor", sensorRange.getRangeDistanceCM());

            telemetry.addData("distance", distance.getDistanceMM());
            telemetry.addData("distance", distance.getDistanceCM());
            telemetry.addData("distance", distance.getDistanceMeter());
            telemetry.addData("distance", distance.getDistanceInch());
        }

        // Stop CV
        if (isStopRequested() || !opModeIsActive()) {
        }
    }
}
