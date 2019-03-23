package org.firstinspires.ftc.teamcode.hardware.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.DrawerSlides;
import org.firstinspires.ftc.teamcode.hardware.Gimbal;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Marker;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.Sensors;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;


/**
 * HardwareMecanum is the class that is used to define all of the hardware for a single robot. In this
 * case, the robot is: Rover Ruckus.
 *
 * This class contains autonomous actions involving multiple mechanisms. These actions
 * may be common to more than one routine.
 */
public class HardwareMecanum extends Mechanism {

    /* Constants */
    private static final int RIGHT_TURN = 90;
    private static final int DIAGONAL_TURN = 45;
    private static final int STRAFE = 6;
    private static final double DRIVE_SPEED = .4;

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
    public Lift lift;
    /**
     * Instance variable containing robot's marker.
     */
    public Marker marker;
    /**
     * Instance variable containing robot's Gimbal.
     */
    public Gimbal gimbal;
    /**
     * Instance variable containing robot's sensors.
     */
    public Sensors sensors;

    /* Miscellaneous mechanisms */

    /**
     * Default constructor for HardwareMain. Instantiates public mechanism instance variables.
     */
    public HardwareMecanum(){
        drivetrain = new Drivetrain();
        drawerSlides = new DrawerSlides();
        acquirer = new Acquirer();
        lift = new Lift();
        marker = new Marker();
        gimbal = new Gimbal();
        sensors = new Sensors();
    }
    /**
     * Overloaded constructor for HardwareMain. Calls the default constructor and sets the OpMode
     * context for the robot.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public HardwareMecanum(LinearOpMode opMode){
        this.opMode = opMode;
        drivetrain = new Drivetrain(opMode);
        drawerSlides = new DrawerSlides(opMode);
        acquirer = new Acquirer(opMode);
        lift = new Lift(opMode);
        marker = new Marker(opMode);
        gimbal = new Gimbal(opMode);
        sensors = new Sensors(opMode);
    }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        acquirer.init(hwMap);
        drawerSlides.init(hwMap);
        lift.init(hwMap);
        marker.init(hwMap);
        gimbal.init(hwMap);
        sensors.init(hwMap);
    }

    /**
     * Initializes drivetrain imu on the robot.
     * @param hwMap     robot's hardware map
     */
    public void imuInit(HardwareMap hwMap) {
        drivetrain.imuInit(hwMap);
    }

    public BNO055IMU.CalibrationStatus imuCalibrated() {
        return drivetrain.singleImu.imuCalibrated();
    }

    public double imuAngle() {
        return drivetrain.imuAngle();
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
            lift.liftToPos(.6, 48.0); // 10 inch
        }
    }

    //strafe RIGHT out of Lander
    public void strafeOutOfLander() {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(.4, -5, 3.0);
        }
    }

    //turn 90 degrees counter clockwise
    public void turn90() {
        if (opMode.opModeIsActive()) {
            drivetrain.turnPID(-RIGHT_TURN);
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
//                drivetrain.strafeToPos(.4,FieldConstants.TILE_HYPOTENUSE / 2.0,4);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
//                drivetrain.strafeToPos(.4,-FieldConstants.TILE_HYPOTENUSE / 2.0,4);
            } else if (location == TensorFlowManager.TFLocation.NONE){
                opMode.telemetry.addData("Detected None", "Robot will take center path");
            }
        }
    }

    //align robot to gold cube and push it into the depot
    public void tfRotateFindGoldLocation(TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE / 3.0, 4);
            if (location == TensorFlowManager.TFLocation.LEFT){
                drivetrain.turnPID(RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE / 2.0, 5);
                drivetrain.turnPID(-RIGHT_TURN);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                //turn right to cube
                drivetrain.turnPID(-RIGHT_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE / 2.0, 5);
                //turn left
                drivetrain.turnPID(RIGHT_TURN);
            } else if (location == TensorFlowManager.TFLocation.NONE){
                opMode.telemetry.addData("Detected None", "Robot will take center path");
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
            drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE + 6, 4);
            if (location == TensorFlowManager.TFLocation.LEFT){
                opMode.telemetry.addData("Sampling from", "left");
                //back
                drivetrain.turnPID(-DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 4);
            } else if (location == TensorFlowManager.TFLocation.CENTER || location == TensorFlowManager.TFLocation.NONE){
                drivetrain.turnPID(-DIAGONAL_TURN);
                opMode.telemetry.addData("Sampling from", "center");
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.turnPID(DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 4);
                drivetrain.turnPID(-RIGHT_TURN);
                opMode.telemetry.addData("Sampling from", "right");
            }
            opMode.telemetry.update();
//            drivetrain.strafeToPos(.3,6, 3);
        }
    }

    /**
     * Sample by driving backwards to hit the sample, then drive forward,
     * then turn clockwise to face the depot
     * depending on where the gold location was, strafe right a certain number of units
     * @param location location of gold cube
     */
    public void tfCraterSamplePID(TensorFlowManager.TFLocation location) {
        if (opMode.opModeIsActive()) {
            //hit sample by moving forward 8 inches
            drivetrain.driveToPos(DRIVE_SPEED, -6, 3);

            opMode.sleep(300);
            //drive backward 8 inches
            drivetrain.driveToPos(DRIVE_SPEED,-6, 3);
            //turn counter-clockwise face depot
            drivetrain.turnPID(RIGHT_TURN);
            //drive forward one floor tile, then correct position by strafing right a certain number of units
            // then go forward again to depot
            if (location == TensorFlowManager.TFLocation.LEFT){
                //if gold was left, strafe right a floor tile
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 4);
                drivetrain.turnPID(DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 4.0, 5);
            } else if (location == TensorFlowManager.TFLocation.CENTER || location == TensorFlowManager.TFLocation.NONE){
                //if gold was center or not found, strafe right 1.5 floor tile
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 1.5, 5);
                drivetrain.turnPID(DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED,-FieldConstants.FLOOR_TILE * 4.0, 5);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                //if gold was right, strafe right 2 floor tiles
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 2, 4);
                drivetrain.turnPID(DIAGONAL_TURN);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 4.0, 5);
            }
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
//        drivetrain.strafeToPos(.5,24,4);

        // scan for mineral
        opMode.sleep(500);
        mineral = visionManager.getDetector();
        if (mineral == TensorFlowManager.TFDetector.GOLD) {
            goldLocation = TensorFlowManager.TFLocation.RIGHT;
        }
        // slide to the right
//        drivetrain.strafeToPos(.5,-48,4);

        // scan for mineral
        opMode.sleep(500);
        mineral = visionManager.getDetector();
        if (mineral == TensorFlowManager.TFDetector.GOLD) {
            goldLocation = TensorFlowManager.TFLocation.LEFT;
        }
//        drivetrain.strafeToPos(.5,24,4);

        return goldLocation;
    }

    public void dropMarker() {
        if (opMode.opModeIsActive()) {
            marker.markerLeft();
            opMode.sleep(500);
        }
    }

    //align to wall after dropping marker
    public void alignToWall(boolean crater) {
        if (opMode.opModeIsActive()) {
            if (crater) {
                //facing crater, strafe left to align to wall
//                drivetrain.strafeToPos(.4,-FieldConstants.FLOOR_TILE / 1.5,4);
            } else {
//                drivetrain.strafeToPos(.4,-16,4);
            }
        }
    }

    public void driveToCrater(boolean crater) {
        if (opMode.opModeIsActive()) {
            if (crater) {
                drivetrain.turn180PID(-180);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 3.5, 5);
            } else {
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.FLOOR_TILE * 3.5, 5);
            }
        }
    }
}
