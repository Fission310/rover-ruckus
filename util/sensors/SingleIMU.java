package org.firstinspires.ftc.teamcode.util.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;

import java.util.Locale;

public class SingleIMU implements IMU {
    public double x_location, y_location, init_heading = 0D;
    public double globalAngle;

    public BNO055IMU imu;
    public AxesOrder axesOrder;
    public Orientation angles;
    public Acceleration gravity;

    public void init(BNO055IMU i, AxesOrder axesOrder, double heading) {
        this.axesOrder = axesOrder;
        this.imu = i;
        this.init_heading = heading;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        if (imu.isGyroCalibrated()) { }

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    //get heading from 0-360 range
    public double getHeading360() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        double heading = (angles.firstAngle + 360) % 360;
        return (heading-init_heading) % 360;
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //returns how off the angle is, returns + if counterclockwise rotation is most efficient, - otherwise
    public double getErrors(double desired) {
        double rawError = ((desired + 360) % 360) - getHeading();
        if (rawError > 180) return rawError - 360;
        return rawError;
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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

    public void resetAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - angles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        angles = angles;

        return globalAngle;
    }
}
