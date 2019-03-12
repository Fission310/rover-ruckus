package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.sensors.MRRangeSensor;
import org.firstinspires.ftc.teamcode.util.sensors.REVDistanceSensor;

/**
 * Sensors is the class that is used to define all of the hardware for a robot's sensors.
 * Sensors must be instantiated, then initialized using <code>init()</code> before being used.
 */
public class Sensors extends Mechanism {

    /* CONSTANTS */
    private final double LEFT_POS = 0;
    private final double NEUTRAL_POS = 0.5;
    private final double RIGHT_POS = 1;

    /* Hardware members */
    public MRRangeSensor frontRange = new MRRangeSensor();
    public MRRangeSensor backRange = new MRRangeSensor();

    public Servo front_servo;
    public Servo back_servo;

    /**
     * Default constructor for Sensors.
     */
    public Sensors(){ }
    /**
     * Overloaded constructor for Sensors. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Sensors(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes gimbal hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve sensors from hardware map and set to initial position (side view)
        frontRange.init(hwMap, RCConfig.FRONT_RANGE_SENSOR);
        backRange.init(hwMap, RCConfig.BACK_RANGE_SENSOR);

        // Retrieve gimbal from hardware map and set to initial position
        front_servo = hwMap.servo.get(RCConfig.FRONT_RIGHT_RANGE);
        back_servo = hwMap.servo.get(RCConfig.BACK_LEFT_RANGE);

        setFrontNeutral();
        setBackNeutral();
    }

    /**
     * Set the gimbal to the left position.
     */
    public void setFrontLeft() {
        front_servo.setPosition(LEFT_POS);
    }
    public void setBackLeft() { back_servo.setPosition(LEFT_POS); }

    /**
     * Set the gimbal to the right position.
     */
    public void setFrontRight() {
        front_servo.setPosition(RIGHT_POS);
    }
    public void setBackRight() { back_servo.setPosition(RIGHT_POS);}

    /**
     * Set the gimbal to the neutral position.
     */
    public void setFrontNeutral() {
        front_servo.setPosition(NEUTRAL_POS);
    }
    public void setBackNeutral() { back_servo.setPosition(NEUTRAL_POS); }

    public void setFrontPosition(double value) {
        front_servo.setPosition(value);
    }
    public void setBackPosition(double value) {
        back_servo.setPosition(value);
    }
}