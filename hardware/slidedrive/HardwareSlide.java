package org.firstinspires.ftc.teamcode.hardware.slidedrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RackNPinonLift;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;


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
//    public Acquirer acquirer;
    /**
     * Instance variable containing robot's linear slides acquirer.
     */
//    public Lift lift;
    /**
     * Instance variable containing robot's rack and pinion lift.
     */
//    public RackNPinonLift rack;
    /**
     * Instance variable containing robot's servo arm.
     */
//    public Arm servoArm;

    /* Miscellaneous mechanisms */

    /**
     * Default constructor for HardwareMain. Instantiates public mechanism instance variables.
     */
    public HardwareSlide(){
        drivetrain = new Drivetrain();
//        acquirer = new Acquirer();
//        lift = new Lift();
//        rack = new RackNPinionLift();
//        servoArm = new Arm();
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
//        acquirer = new Acquirer(opMode);
//        lift = new Lift(opMode);
//        rack = new RackNPinionLift(opMode);
//        servoArm = new Arm(opMode);
    }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
//        acquirer.init(hwMap);
//        lift.init(hwMap);
//        rack.init(hwMap);
//        servoArm.init(hwMap);
    }

    /**
     * Waits for opMode's to start. Can perform actions while waiting.
     */
    public void waitForStart() {
        while (!opMode.opModeIsActive() && !opMode.isStopRequested()) {
            opMode.telemetry.addData("Heading:", drivetrain.getHeading());
            opMode.telemetry.update();
        }
    }

    /**
     * Autonomous action for sampling the gold cube. Uses the robot's servo arm mechanism to detect gold cube
     * and in a linear slide fashion.
     * This assumes the vision sensor faces the back of the robot.
     *
     *  @param visionManager    VisionManager containing the GoldDetector
     */

    public void samplePID(VisionManager visionManager) {
        if (opMode.opModeIsActive()) {
            while (!visionManager.isGoldAligned()) {
                drivetrain.drive(0.25, 0.0);
            }
            if (visionManager.isGoldAligned()) {
                drivetrain.driveToPos(0.3, -FieldConstants.TILE_HYPOTENUSE / 6, -FieldConstants.TILE_HYPOTENUSE / 6, 2);
                drivetrain.turnPID(-90);
                drivetrain.driveToPos(0.3, FieldConstants.FLOOR_TILE * 6, FieldConstants.FLOOR_TILE * 6, 3);
                drivetrain.driveToPos(0.3, -FieldConstants.FLOOR_TILE * 6, -FieldConstants.FLOOR_TILE * 6, 3);
                drivetrain.turnPID(90);
            }
        }
    }

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
