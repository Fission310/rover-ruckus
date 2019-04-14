package org.firstinspires.ftc.teamcode.util.sensors;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ma3Encoder{
    public AnalogInput input;
    private static final double A = .3928;
    private static final double B = -1.66868;

    public Ma3Encoder(AnalogInput input) { this.input = input; }
}
