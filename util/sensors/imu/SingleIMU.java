package org.firstinspires.ftc.teamcode.util.sensors.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * SingleIMU handles all methods pertaining to the built in IMU in the REV Expansion Hub.
 */
public class SingleIMU {
    public double x_location, y_location, init_heading = 0D;
    public double globalAngle;
    public static double startingAngle;

    public BNO055IMU imu;
    public AxesOrder axesOrder;
    public Orientation lastAngles = new Orientation();
    public Acceleration acceleration;
    public Position position;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public SingleIMU() { }

    public void init(HardwareMap hwMap, AxesOrder axesOrder, double heading) {
        this.axesOrder = axesOrder;
        this.imu = hwMap.get(BNO055IMU.class, "imu");
        this.init_heading = heading;

        /**
         * In the IMU mode the relative orientation of the BNO055
         * in space is calculated from the accelerometer and gyroscope data. The calculation is fast
         * (i.e. high output data rate).
         */
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new NaiveAccelerationIntegrator();

        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 200);
    }

    public BNO055IMU.CalibrationStatus imuCalibrated() {
        return imu.getCalibrationStatus();
    }

    /**
     * Resets starting angle because startingAngle is a static variable and must be cleared for every use.
     */
    public void setStartingAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startingAngle = angles.firstAngle;
    }

    /**
     * Resets starting angle because startingAngle is a static variable and must be cleared for every use.
     */
    public void resetStartingAngle() {
        startingAngle = 0;
    }

    /**
     * Returns the change in angular rotation from the robots current position to starting position(auton).
     */
    public double getDeltaStartingAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // calculates the difference from when we first initialized the rotation to where we are at now
        double deltaAngle = angles.firstAngle - startingAngle;

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        globalAngle += deltaAngle;

        // set new last angle to current angle
        lastAngles = angles;

        double returnAngle = globalAngle;
        resetAngle();
        return returnAngle;
    }

    /**
     * Returns the z axis for rotation.
     */
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    /**
     * Returns the linear acceleration of the robot.
     */
    public double getXAxis() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }
    /**
     * Returns the y axis for rotation.
     */
    public double getYAxis() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }
    /**
     * Returns the x acceleration.
     */
    public double getXAccel() {
        acceleration = imu.getAcceleration();
        return acceleration.xAccel;
    }
    /**
     * Returns the y axis acceleration.
     */
    public double getaYAccel() {
        acceleration = imu.getAcceleration();
        return acceleration.yAccel;
    }

    /**
     * Returns the distance traveled in the x axis.
     */
    public double getXDistance() {
        position = imu.getPosition();
        return position.x;
    }
    /**
     * Returns the distance traveled in the y axis.
     */
    public double getYDistance() {
        position = imu.getPosition();
        return position.y;
    }
    /**
     * Returns the distance traveled in the z axis.
     */
    public double getZDistance() {
        position = imu.getPosition();
        return position.z;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // calculates the difference from when we first initialized the rotation to where we are at now
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        globalAngle += deltaAngle;

        // set new last angle to current angle
        lastAngles = angles;

        return globalAngle;
    }

    public double getError(double targetAngle) {
        double heading = getHeading();
        if (targetAngle > heading) {
            if (targetAngle - heading > 180) {
                return 360 - Math.abs(targetAngle) - Math.abs(heading);
            } else {
                return targetAngle - heading;
            }
        } else {
            if (targetAngle - heading > 180) {
                return -(360 - Math.abs(targetAngle) - Math.abs(heading));
            } else {
                return heading - targetAngle;
            }
        }
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
