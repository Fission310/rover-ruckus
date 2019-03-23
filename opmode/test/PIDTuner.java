package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.mecanum.HardwareMecanum;
import org.firstinspires.ftc.teamcode.util.motion.PIDController;
import org.firstinspires.ftc.teamcode.util.signals.BackgroundColorManager;

import static java.lang.Math.abs;

/**
 * PIDTuner is a dedicated teleop for tuning the drivetrain's PID Control constants for P, I, and D.
 */
@TeleOp(name = "PID Tuner", group = "Test")
public class PIDTuner extends OpMode {

    /* Private OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware map */
    private HardwareMecanum robot = new HardwareMecanum();

    double coefDValue = 0.00;
    double coefIValue = 0.00;
    double coefPValue = 0.00;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.drivetrain.encoderInit();
        robot.imuInit(hardwareMap);
    }

    /**
     * Runs continuously while OpMode is waiting to start.
     * @see OpMode#init_loop()
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status:", "Waiting to start");
        telemetry.addData("Gyro Is Calibrated", robot.imuCalibrated());
        telemetry.addData("imu angle",robot.imuAngle());
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

        /**
         * Gamepad 1
         */
        if(gamepad1.x) {
            robot.drivetrain.turnPID(90);
        }

        if(gamepad1.y) {
            robot.drivetrain.turnPID(-90);
        }

        if (gamepad1.dpad_up) {
            coefPValue += .010;
            robot.drivetrain.pidRotate = new PIDController(coefPValue, coefIValue, coefDValue);
        }
        if (gamepad1.dpad_down) {
            coefPValue -= .010;
            robot.drivetrain.pidRotate = new PIDController(coefPValue, coefIValue, coefDValue);
        }

        if (gamepad1.dpad_right) {
            coefDValue += .010;
            robot.drivetrain.pidRotate = new PIDController(coefPValue, coefIValue, coefDValue);
        }
        if (gamepad1.dpad_left) {
            coefDValue -= .010;
            robot.drivetrain.pidRotate = new PIDController(coefPValue, coefIValue, coefDValue);
        }

        if (gamepad2.dpad_up) {
            coefIValue += .010;
            robot.drivetrain.pidRotate = new PIDController(coefPValue, coefIValue, coefDValue);
        }
        if (gamepad2.dpad_down) {
            coefIValue -= .010;
            robot.drivetrain.pidRotate = new PIDController(coefPValue, coefIValue, coefDValue);
        }
        /**
         * Telemetry
         */
        telemetry.addData("PID Turn Coeff: ", "P: %.2f | I: %.2f | D: %.2f",
                robot.drivetrain.pidRotate.getP(),
                robot.drivetrain.pidRotate.getI(),
                robot.drivetrain.pidRotate.getD());
        telemetry.addData("IMU angle: ", robot.imuAngle());
        double[] positions = robot.drivetrain.getPositions();
        telemetry.addData("Encoder counts: ", "lf: %.2f | rf: %.2f | lb: %.2f | rb: %.2f",
                positions[0],
                positions[1],
                positions[2],
                positions[3]);
    }

    @Override
    public void stop() { }
}