package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Hopper_Drawer is the class that is used to define all of the hardware for a robot's hopper + drawer slides lift.
 * Hopper_Drawer must be instantiated, then initialized using <code>init()</code> before being used.
 *
 * This class also contains autonomous actions involving the hopper + drawer slides lift.
 */
public class Hopper_Drawer extends Mechanism {

    /* Hardware members */
    private DcMotor drawerSlide;

    /**
     * Default constructor for Hopper_Drawer.
     */
    public Hopper_Drawer(){ }

    /**
     * Overloaded constructor for Hopper_Drawer. Sets the OpMode context.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public Hopper_Drawer(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initializes drawer slides hardware.
     * @param hwMap        robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        // Retrieve servos from hardware map and assign to instance vars
        drawerSlide = hwMap.dcMotor.get(RCConfig.DRAWER_SLIDES);

        // Set braking behavior
        drawerSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set polarity
        drawerSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power
        drawerSlide.setPower(0);
    }

    public void encoderInit() {
        drawerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drawerSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDrawerSlideZeroPowers(DcMotor.ZeroPowerBehavior behavior) { drawerSlide.setZeroPowerBehavior(behavior); }

    /**
     * Sets power for linear slides motors.
     * @param power        Motor power with range of (-1 to 1)
     */
    public void setDrawerSlidePower(double power) {
        drawerSlide.setPower(power);
    }

    public double[] getPositions() {
        double[] positions = new double[1];
        positions[0] = drawerSlide.getCurrentPosition();

        return positions;
    }
}