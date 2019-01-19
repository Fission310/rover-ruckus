package org.firstinspires.ftc.teamcode.hardware.slidedrive;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.Marker;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RackNPinonLift;
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
     * Instance variable containing robot's rack and pinion lift.
     */
//    public RackNPinonLift rack;
    /**
     * Instance variable containing robot's marker.
     */
//    public Marker marker;

    /* Miscellaneous mechanisms */

    /**
     * Default constructor for HardwareMain. Instantiates public mechanism instance variables.
     */
    public HardwareSlide(){
        drivetrain = new Drivetrain();
//        acquirer = new Acquirer();
//        rack = new RackNPinonLift();
//        marker = new Marker();
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
//        rack = new RackNPinonLift(opMode);
//        marker = new Marker(opMode);
    }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
//        acquirer.init(hwMap);
//        rack.init(hwMap);
//        marker.init(hwMap);
    }

    /**
     * Initializes drivetrain imu on the robot.
     * @param hwMap     robot's hardware map
     */
    public void imuInit(HardwareMap hwMap) {
        drivetrain.imuInit(hwMap);
    }

    /**
     * Waits for opMode's to start. Can perform actions while waiting.
     */
    public void waitForStart() {
        while (!opMode.opModeIsActive() && !opMode.isStopRequested()) {
            opMode.telemetry.addData("Heading:", drivetrain.singleImu.getHeading());
            opMode.telemetry.update();
        }
    }

    /**
     * Autonomous action for landing the robot using the rack and pinion mechanism.
     */
    public void land() {
        if (opMode.opModeIsActive()) {
//            rack.rackToPos(.4, FieldConstants.HANG_HEIGHT);
        }
    }

    public void turn90() {
        if (opMode.opModeIsActive()) {
            drivetrain.turnPID(-90);
        }
    }
    /**
     * Autonomous action for finding the location of the gold cube.
     * This assumes the vision sensor faces the slide of the robot.
     *
     *  @param visionManager    VisionManager containing the GoldDetector
     *  @param location    location holds the TFLocation detected
     */
    public void findGoldLocation(VisionManager visionManager, SamplingOrderDetector.GoldLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.3, -FieldConstants.TILE_HYPOTENUSE, -FieldConstants.TILE_HYPOTENUSE, 5);

            if (location == location.LEFT){
                drivetrain.turnPID(-90);
                drivetrain.driveToPos(.3,FieldConstants.TILE_HYPOTENUSE / 2, FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(90);
            } else if (location == location.CENTER){

            } else if (location == location.RIGHT){
                drivetrain.turnPID(90);
                drivetrain.driveToPos(.3,FieldConstants.TILE_HYPOTENUSE / 2, FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(-90);
            } else if (location == location.UNKNOWN){
                opMode.telemetry.addData("Detected None", "Value" + location);
            }
        }
    }

    /**
     * Autonomous action for sampling the gold cube. Uses the robot's servo arm mechanism to detect gold cube
     * and in a linear slide fashion.
     * This assumes the vision sensor faces the slide of the robot.
     *
     *  @param visionManager    VisionManager containing the GoldDetector
     *  @param location    location holds the TFLocation detected
     */
    public void samplePID(VisionManager visionManager, SamplingOrderDetector.GoldLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.3, -FieldConstants.TILE_HYPOTENUSE, -FieldConstants.TILE_HYPOTENUSE, 5);
            if (location == location.LEFT){
                drivetrain.turnPID(45);
                drivetrain.driveToPos(.3, FieldConstants.FLOOR_TILE, FieldConstants.FLOOR_TILE, 5);
            } else if (location == location.CENTER || location == location.UNKNOWN){
                drivetrain.turnPID(-45);
            } else if (location == location.RIGHT){
                drivetrain.turnPID(45);
                drivetrain.driveToPos(.3, FieldConstants.TILE_HYPOTENUSE, FieldConstants.TILE_HYPOTENUSE, 5);
                drivetrain.turnPID(90);
            }
        }
    }

    /**
     * Autonomous action for dropping the marker. Uses the robot's distance sensor to detect the robot's
     * position using the vuforia pictograph. Moves parallel to wall until the edge is
     * reached.
     *
//     *  @param targetCol      the cryptobox column that is being targeted (left is 0, center is 1, right is 2)
//     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */
    public void dropMarker() {
        if (opMode.opModeIsActive()) {
//            marker.markerLeft();
            drivetrain.driveToPos(.5, 12, 12, 5);
            drivetrain.driveToPos(.5, -12, -12, 5);
        }
    }

    public void alignToWall() {
        if (opMode.opModeIsActive()) { }
    }

    public void driveToCrater() {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.5, FieldConstants.TILE_HYPOTENUSE * 3.5, FieldConstants.TILE_HYPOTENUSE * 3.5, 5);

        }
    }
}
