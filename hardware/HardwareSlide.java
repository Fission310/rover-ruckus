package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.legacy.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.legacy.Lift;
import org.firstinspires.ftc.teamcode.hardware.legacy.Tape;
import org.firstinspires.ftc.teamcode.hardware.slidedrive.Drivetrain;


/**
 * HardwareTank is the class that is used to define all of the hardware for a single robot. In this
 * case, the robot is: Rover Ruckus.
 *
 * This class contains autonomous actions involving multiple mechanisms. These actions
 * may be common to more than one routine.
 */
public class HardwareSlide extends Mechanism {

    /* Mechanisms */
    /**
     * Instance variable containing robot's drivetrain.
     */
    public Drivetrain drivetrain;
    /**
     * Instance variable containing robot's acquirer.
     */
    public Acquirer acquirer;
    /**
     * Instance variable containing robot's lift.
     */
    public Lift lift;
    /**
     * Instance variable containing robot's tape measure.
     */
    public Tape tape;
    /**
     * Instance variable containing robot's servo arm.
     */
    public Arm servoArm;

    /* Miscellaneous mechanisms */

    /**
     * Default constructor for HardwareMain. Instantiates public mechanism instance variables.
     */
    public HardwareSlide(){
        drivetrain = new Drivetrain();
        acquirer = new Acquirer();
        lift = new Lift();
        tape = new Tape();
        servoArm = new Arm();
    }
    /**
     * Overloaded constructor for HardwareMain. Calls the default constructor and sets the OpMode
     * context for the robot.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public HardwareSlide(LinearOpMode opMode){
        this.opMode = opMode;
        drivetrain = new Drivetrain(opMode);
        acquirer = new Acquirer(opMode);
        lift = new Lift(opMode);
        tape = new Tape(opMode);
        servoArm = new Arm(opMode);
    }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        acquirer.init(hwMap);
        lift.init(hwMap);
        tape.init(hwMap);
        servoArm.init(hwMap);
    }

    /**
     * Waits for opMode's to start. Can perform actions while waiting.
     */
    public void waitForStart() {
        while (!opMode.isStarted()) {
            opMode.telemetry.addData("Heading:", drivetrain.getHeading());
            opMode.telemetry.update();
        }
    }

    /**
     * Autonomous action for sampling the gold cube. Uses the robot's servo arm mechanism to detect gold cube
     * and in a linear slide fashion.
     * This assumes the color sensor faces the back of the robot.
     *
     *  @param visionManager    VisionManager containing the GoldDetector
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */

    // gold detector

    //

    /**
     * Autonomous action for dropping the marker. Uses the robot's distance sensor to detect the robot's
     * position using the vuforia pictograph. Moves parallel to wall until the edge is
     * reached.
     *
     *  @param targetCol      the cryptobox column that is being targeted (left is 0, center is 1, right is 2)
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */

    // Marker scorer

    //
}
