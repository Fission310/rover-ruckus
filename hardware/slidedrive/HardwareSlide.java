package org.firstinspires.ftc.teamcode.hardware.slidedrive;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.DrawerSlides;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RackNPinonLift;
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

    /* Constants */
    private static final int RIGHT_TURN = 88;
    private static final int DIAGONAL_TURN = 43;
    private static final int STRAFE = 6;
    private static final double DRIVE_SPEED = .3;

    /* Mechanisms */
    /**
     * Instance variable containing robot's drivetrain.
     */
    public Drivetrain drivetrain;
    /**
     * Instance variable containing robot's drawer slides.
     */
    public DrawerSlides drawerSlides;
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
        drawerSlides = new DrawerSlides();
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
        drawerSlides = new DrawerSlides(opMode);
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
        drawerSlides.init(hwMap);
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
//            rack.rackToPos(.8, -FieldConstants.HANG_HEIGHT);
        }
    }

    public void turn90() {
        if (opMode.opModeIsActive()) {
            drivetrain.turnPID(-RIGHT_TURN);
        }
    }
    /**
     * Autonomous action for finding the location of the gold cube.
     * This assumes the vision sensor faces the slide of the robot.
     *
     *  @param visionManager    VisionManager containing the GoldDetector
     *  @param location    location holds the TFLocation detected
     */
    public void depotFindGoldLocation(VisionManager visionManager, SamplingOrderDetector.GoldLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE, 5);

            if (location == location.LEFT){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
                drivetrain.turnPID(RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(-RIGHT_TURN);
            } else if (location == location.CENTER){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
            } else if (location == location.RIGHT){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
                drivetrain.turnPID(-RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(RIGHT_TURN);
            } else if (location == location.UNKNOWN){
                opMode.telemetry.addData("Detected None", "Value" + location);
            }
        }
    }
    public void tfDepotFindGoldLocation(TensorFlowManager visionManager, TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE, 5);

            if (location == location.LEFT){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
                drivetrain.turnPID(RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(-RIGHT_TURN);
            } else if (location == location.CENTER){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
            } else if (location == location.RIGHT){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
                drivetrain.turnPID(-RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(RIGHT_TURN);
            } else if (location == location.NONE){
                opMode.telemetry.addData("Detected None", "Value" + location);
            }
        }
    }
    public void craterFindGoldLocation(VisionManager visionManager, SamplingOrderDetector.GoldLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE, 5);

            if (location == location.LEFT){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
                drivetrain.turnPID(RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(-RIGHT_TURN);
            } else if (location == location.CENTER){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
            } else if (location == location.RIGHT){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
                drivetrain.turnPID(-RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(RIGHT_TURN);
            } else if (location == location.UNKNOWN){
                opMode.telemetry.addData("Detected None", "Value" + location);
            }
        }
    }

    public void tfCraterFindGoldLocation(TensorFlowManager visionManager, TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE, 5);

            if (location == location.LEFT){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
                drivetrain.turnPID(RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(-RIGHT_TURN);
            } else if (location == location.CENTER){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
            } else if (location == location.RIGHT){
                opMode.telemetry.addData("Detected Gold", "Value" + location);
                drivetrain.turnPID(-RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE / 2, 5);
                drivetrain.turnPID(RIGHT_TURN);
            } else if (location == location.NONE){
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
    public void depotSamplePID(VisionManager visionManager, SamplingOrderDetector.GoldLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE, 5);
            if (location == location.LEFT){
                drivetrain.turnPID(DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 5);
                drivetrain.turnPID(-RIGHT_TURN);
            } else if (location == location.CENTER || location == location.UNKNOWN){
                drivetrain.turnPID(-DIAGONAL_TURN);
            } else if (location == location.RIGHT){
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE, 5);
            }
        }
    }

    public void tfDepotSamplePID(TensorFlowManager visionManager, TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE + STRAFE, 5);
            if (location == location.LEFT){
                drivetrain.turnPID(DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 5);
                drivetrain.turnPID(-RIGHT_TURN);
            } else if (location == location.CENTER || location == location.NONE){
                drivetrain.turnPID(-DIAGONAL_TURN);
            } else if (location == location.RIGHT){
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE, 5);
            }
        }
    }

    public void craterSamplePID(VisionManager visionManager, SamplingOrderDetector.GoldLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED,-FieldConstants.TILE_HYPOTENUSE / 3.0, 5);
            if (location == location.LEFT){
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 3.0, 5);
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE, 5);
                drivetrain.turnPID(DIAGONAL_TURN);
            } else if (location == location.CENTER || location == location.UNKNOWN){
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 3.0, 5);
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE * 1.5, 5);
                drivetrain.turnPID(DIAGONAL_TURN);
            } else if (location == location.RIGHT){
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 3.0, 5);
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE * 2.0, 5);
                drivetrain.turnPID(DIAGONAL_TURN);
            }
        }
    }

    public void tfCraterSamplePID(TensorFlowManager visionManager, TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED,-FieldConstants.TILE_HYPOTENUSE / 3.0, 5);
            if (location == location.LEFT){
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 3.0, 5);
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE, 5);
                drivetrain.turnPID(DIAGONAL_TURN);
            } else if (location == location.CENTER || location == location.NONE){
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 3.0, 5);
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE * 1.5, 5);
                drivetrain.turnPID(DIAGONAL_TURN);
            } else if (location == location.RIGHT){
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE / 3.0, 5);
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED,FieldConstants.TILE_HYPOTENUSE * 2.0, 5);
                drivetrain.turnPID(DIAGONAL_TURN);
            }
        }
    }

    public void driveToDepot() {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.5,FieldConstants.FLOOR_TILE * 3.0, 5);

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
        }
    }

    public void craterDropMarker() {
        if (opMode.opModeIsActive()) {
            drivetrain.turnPID(180);
//            marker.markerLeft();

        }
    }

    public void alignToWall() {
        if (opMode.opModeIsActive()) { }
    }

    public void driveToCrater() {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.5,FieldConstants.FLOOR_TILE * 3.5, 7);
        }
    }
    public void craterDriveToCrater() {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.5,-FieldConstants.FLOOR_TILE * 4, 7);
        }
    }
}
