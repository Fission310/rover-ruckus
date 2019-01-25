package org.firstinspires.ftc.teamcode.util.sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MRRangeSensor {

    public ModernRoboticsI2cRangeSensor rangeSensor;

    public MRRangeSensor() { }

    public void init(HardwareMap hwMap, String deviceName) {
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, deviceName);
    }

    public double getRangeRawUltrasonic() { return rangeSensor.rawUltrasonic(); }
    public double getRangeRawOptical() { return rangeSensor.rawOptical(); }
    public double getRangeOpticalCM() { return rangeSensor.cmOptical(); }
    public double getRangeDistanceCM() { return rangeSensor.getDistance(DistanceUnit.CM); }
}
