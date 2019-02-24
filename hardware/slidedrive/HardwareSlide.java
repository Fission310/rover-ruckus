package org.firstinspires.ftc.teamcode.hardware.slidedrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.DrawerSlides;
import org.firstinspires.ftc.teamcode.hardware.Marker;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;


/**
 * HardwareTank is the class that is used to define all of the hardware for a single robot. In this
 * case, the robot is: Rover Ruckus.
 *
 * This class contains autonomous actions involving multiple mechanisms. These actions
 * may be common to more than one routine.
 */
public class HardwareSlide extends Mechanism {

    /* Constants */
    private static final int RIGHT_TURN = 90;
    private static final int DIAGONAL_TURN = 45;
    private static final int STRAFE = 6;
    private static final double DRIVE_SPEED = .5;

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
    public Acquirer acquirer;
    /**
     * Instance variable containing robot's lift and pinion lift.
     */
//    public Lift lift;
    /**
     * Instance variable containing robot's marker.
     */
    public Marker marker;

    /* Miscellaneous mechanisms */

    /**
     * Default constructor for HardwareMain. Instantiates public mechanism instance variables.
     */
    public HardwareSlide(){
        drivetrain = new Drivetrain();
        drawerSlides = new DrawerSlides();
        acquirer = new Acquirer();
//        lift = new Lift();
        marker = new Marker();
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
        acquirer = new Acquirer(opMode);
//        lift = new Lift(opMode);
        marker = new Marker(opMode);
    }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        acquirer.init(hwMap);
        drawerSlides.init(hwMap);
//        lift.init(hwMap);
        marker.init(hwMap);
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
     * Autonomous action for landing the robot using the lift and pinion mechanism.
     */
    public void land() {
        if (opMode.opModeIsActive()) {
           //lift.liftToPos(.3, 7.5);
        }
    }

    public void strafeOutOfLander() {
        if (opMode.opModeIsActive()) {
//            drivetrain.driveToPos(.4, 4, 3.0);
            drivetrain.strafeToPos(.5, FieldConstants.TILE_HYPOTENUSE / 2, 3);
//            drivetrain.driveToPos(.4, -4, 3.0);
        }
    }

    public void turn90() {
        if (opMode.opModeIsActive()) {
            drivetrain.turnPID(RIGHT_TURN);
        }
    }
    /**
     * Autonomous action for finding the location of the gold cube.
     * This assumes the vision sensor faces the slide of the robot.
     *
     *  @param location    location holds the TFLocation detected
     */
    public void tfFindGoldLocation(TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE / 3.0, 4);
            if (location == TensorFlowManager.TFLocation.LEFT){
                drivetrain.strafeToPos(.4,FieldConstants.TILE_HYPOTENUSE / 2.0,4);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.strafeToPos(.4,-FieldConstants.TILE_HYPOTENUSE / 2.0,4);
            } else if (location == TensorFlowManager.TFLocation.NONE){
                opMode.telemetry.addData("Detected None", "Robot will take center path");
            }
        }
    }

    //align robot to gold cube and push it into the depot
    public void tfRotateFindGoldLocation(TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE / 3.0, 4);
            if (location == TensorFlowManager.TFLocation.LEFT){
                drivetrain.turnPID(-RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE / 2.0, 5);
                drivetrain.turnPID(RIGHT_TURN);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.turnPID(RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE / 2.0, 5);
                drivetrain.turnPID(-RIGHT_TURN);
            } else if (location == TensorFlowManager.TFLocation.NONE){
                opMode.telemetry.addData("Detected None", "Robot will take center path");
            }
        }
    }

    public void rotFindGoldLocation(TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            if (location == TensorFlowManager.TFLocation.LEFT){
                drivetrain.turnPID(20);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.turnPID(-20);
            } else if (location == TensorFlowManager.TFLocation.NONE){
                opMode.telemetry.addData("Detected None", "Robot will take center path");
            }
            drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE / 3.0, 4);
            if (location == TensorFlowManager.TFLocation.LEFT){
                drivetrain.turnPID(-20);
                drivetrain.strafeToPos(.4,-FieldConstants.TILE_HYPOTENUSE / 3.0,4);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.turnPID(20);
                drivetrain.strafeToPos(.4,FieldConstants.TILE_HYPOTENUSE / 3.0,4);
            }
        }
    }

    /**
     * Autonomous action for sampling the gold cube. Uses the robot's servo arm mechanism to detect gold cube
     * and in a linear slide fashion.
     * This assumes the vision sensor faces the slide of the robot.
     *
     *  @param location    location holds the TFLocation detected
     */
    public void tfDepotSamplePID(TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.TILE_HYPOTENUSE + 6, 4);
            if (location == TensorFlowManager.TFLocation.LEFT){
                opMode.telemetry.addData("Sampling from", "left");
                //back
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.FLOOR_TILE, 4);
                drivetrain.turnPID(RIGHT_TURN);

            } else if (location == TensorFlowManager.TFLocation.CENTER || location == TensorFlowManager.TFLocation.NONE){
                drivetrain.turnPID(DIAGONAL_TURN);
                opMode.telemetry.addData("Sampling from", "center");
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.turnPID(DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.FLOOR_TILE, 4);
                opMode.telemetry.addData("Sampling from", "right");
            }
            opMode.telemetry.update();

            drivetrain.strafeToPos(.4,-FieldConstants.TILE_HYPOTENUSE / 1.6, 4);
        }
    }

    public void tfCraterSamplePID(TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            //hit sample
            drivetrain.driveToPos(DRIVE_SPEED, 8, 3);

            opMode.sleep(300);
            //drive forward again
            drivetrain.driveToPos(DRIVE_SPEED,-8, 3);
            //turn clockwise face depot
            drivetrain.turnPID(-DIAGONAL_TURN);
            if (location == TensorFlowManager.TFLocation.LEFT){
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 5);
                drivetrain.strafeToPos(DRIVE_SPEED, FieldConstants.FLOOR_TILE, 5);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 3.0, 5);
            } else if (location == TensorFlowManager.TFLocation.CENTER || location == TensorFlowManager.TFLocation.NONE){
                //drive forward one floor tile, then correct position by strafing right, then go forward again to depot
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 5);
                drivetrain.strafeToPos(DRIVE_SPEED, FieldConstants.FLOOR_TILE * 1.5, 5);
                drivetrain.driveToPos(DRIVE_SPEED,-FieldConstants.FLOOR_TILE * 3.0, 5);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.driveToPos(DRIVE_SPEED,-FieldConstants.FLOOR_TILE * 1.5, 5);
                drivetrain.strafeToPos(DRIVE_SPEED, FieldConstants.FLOOR_TILE * 2.0, 5);
                drivetrain.driveToPos(DRIVE_SPEED,-FieldConstants.FLOOR_TILE * 3.0, 5);
            }
            //turn around face crater
            drivetrain.turn180PID(179);
        }
    }

    public TensorFlowManager.TFLocation tfSlideSample(TensorFlowManager visionManager, TensorFlowManager.TFDetector mineral) {
        TensorFlowManager.TFLocation goldLocation = TensorFlowManager.TFLocation.NONE;
        mineral = visionManager.getDetector();
        if (mineral == TensorFlowManager.TFDetector.GOLD) {
            goldLocation = TensorFlowManager.TFLocation.CENTER;
        }

        // drive forward
        drivetrain.driveToPos(.4, -7.0,3.0);
        // slide to the left
        drivetrain.strafeToPos(.5,24,4);

        // scan for mineral
        opMode.sleep(500);
        mineral = visionManager.getDetector();
        if (mineral == TensorFlowManager.TFDetector.GOLD) {
            goldLocation = TensorFlowManager.TFLocation.RIGHT;
        }
        // slide to the right
        drivetrain.strafeToPos(.5,-48,4);

        // scan for mineral
        opMode.sleep(500);
        mineral = visionManager.getDetector();
        if (mineral == TensorFlowManager.TFDetector.GOLD) {
            goldLocation = TensorFlowManager.TFLocation.LEFT;
        }
        drivetrain.strafeToPos(.5,24,4);

        return goldLocation;
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
            marker.markerLeft();
            opMode.sleep(500);
        }
    }

    public void alignToWall(boolean crater) {
        if (opMode.opModeIsActive()) {
            if (crater) {
                //facing crater, strafe left to align to wall
                drivetrain.strafeToPos(.4,-FieldConstants.FLOOR_TILE / 1.5,4);
            } else {
                drivetrain.strafeToPos(.4,-16,4);
            }
        }
    }

    public void driveToCrater(boolean crater) {
        if (opMode.opModeIsActive()) {
            if (crater) {
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 3.5, 5);
            } else {
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 3.5, 5);
            }
        }
    }
}
