package org.firstinspires.ftc.teamcode.util.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public interface IMU {
    void init(BNO055IMU f, AxesOrder axesOrder, double heading);
    double getHeading();
    double getError(double d);
}
