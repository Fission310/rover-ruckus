package org.firstinspires.ftc.teamcode.hardware.slidedrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RackNPinonLift;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;
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

    public void land() {
        if (opMode.opModeIsActive()) {

        }
    }

    public void findGoldLocation(TensorFlowManager visionManager, TensorFlowManager.TFLocation location) {
        TensorFlowManager.TFLocation newLocation;

        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.3, FieldConstants.TILE_HYPOTENUSE, FieldConstants.TILE_HYPOTENUSE, 3);

            if (location == TensorFlowManager.TFLocation.LEFT){
                drivetrain.strafeToPos(.3,-FieldConstants.TILE_HYPOTENUSE / 2, 3);
            } else if (location == TensorFlowManager.TFLocation.CENTER){

            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.strafeToPos(.3,FieldConstants.TILE_HYPOTENUSE / 2, 3);
            } else if (location == TensorFlowManager.TFLocation.NONE){
                newLocation = visionManager.getLocation();
                findGoldLocation(visionManager, newLocation);
            }
        }
    }

    /**
     * Autonomous action for sampling the gold cube. Uses the robot's servo arm mechanism to detect gold cube
     * and in a linear slide fashion.
     * This assumes the vision sensor faces the back of the robot.
     *
     *  @param visionManager    VisionManager containing the GoldDetector
     */
    public void samplePID(TensorFlowManager visionManager, TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.3, FieldConstants.TILE_HYPOTENUSE, FieldConstants.TILE_HYPOTENUSE, 3);
            if (location == TensorFlowManager.TFLocation.LEFT){
                drivetrain.turnPID(-45);
                drivetrain.driveToPos(.3, FieldConstants.TILE_HYPOTENUSE, FieldConstants.TILE_HYPOTENUSE, 3);
            } else if (location == TensorFlowManager.TFLocation.CENTER){

            } else if (location == TensorFlowManager.TFLocation.RIGHT){

            }
        }
    }

    public void dropMarker() {
        if (opMode.opModeIsActive()) {

        }
    }

    public void alignToWall() {
        if (opMode.opModeIsActive()) {

        }
    }

    public void driveToCrater() {
        if (opMode.opModeIsActive()) {

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
