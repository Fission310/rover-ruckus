package org.firstinspires.ftc.teamcode.hardware.mecanum;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Acquirer_Slides;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.Gimbal;
import org.firstinspires.ftc.teamcode.hardware.Hopper_Drawer;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.LiftConstants;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.util.vision.TensorFlowManager;

import java.util.HashMap;
import java.util.Map;


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
    private static final double DRIVE_SPEED = .4;
    private boolean updated = false;

    /* Mechanisms */
    /**
     * Instance variable containing robot's drivetrain.
     */
    public Drivetrain drivetrain;
    /**
     * Instance variable containing robot's acquirer.
     */
    public Acquirer_Slides acquirer;
    /**
     * Instance variable containing robot's acquirer.
     */
    public Hopper_Drawer hopper;
    /**
     * Instance variable containing robot's lift and pinion lift.
     */
    public Lift lift;
    /**
     * Instance variable containing robot's Gimbal.
     */
    public Gimbal gimbal;
    /**
     * Instance variable containing robot's sensors.
     */
//    public Sensors sensors;

    /* Miscellaneous mechanisms */

    private Map<DcMotorController, LynxModuleIntf> hubs;
    private Map<LynxModuleIntf, LynxGetBulkInputDataResponse> responses;

    /**
     * Default constructor for HardwareMain. Instantiates public mechanism instance variables.
     */
    public HardwareMecanum(){
        try { drivetrain = new Drivetrain(); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with drivetrain");
        }
        try { acquirer = new Acquirer_Slides(); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with acquirer");
        }
        try { hopper = new Hopper_Drawer(); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with hopper");
        }
        try { lift = new Lift(); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with lift");
        }
        try { gimbal = new Gimbal(); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with gimbal");
        }
//        try { sensors = new Sensors(); }
//        catch (Exception e) {
//            opMode.telemetry.addData("Status", "Problem with sensors");
//        }
    }
    /**
     * Overloaded constructor for HardwareMain. Calls the default constructor and sets the OpMode
     * context for the robot.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public HardwareMecanum(LinearOpMode opMode){
        this.opMode = opMode;
        try { drivetrain = new Drivetrain(opMode); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with drivetrain");
        }
        try { acquirer = new Acquirer_Slides(opMode); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with acquirer");
        }
        try { hopper = new Hopper_Drawer(opMode); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with hopper");
        }
        try { lift = new Lift(opMode); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with lift");
        }
        try { gimbal = new Gimbal(opMode); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "Problem with gimbal");
        }
//        try { sensors = new Sensors(opMode); }
//        catch (Exception e) {
//            opMode.telemetry.addData("Status", "Problem with sensors");
//        }
 }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        try {
            drivetrain.init(hwMap);
            drivetrain.encoderInit();
        } catch (Exception e) {
            opMode.telemetry.addData("Status", "problem initializing drivetrain");
        }
        try { acquirer.init(hwMap); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "problem initializing acquirer");
        }
        try { hopper.init(hwMap); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "problem initializing hopper");
        }
        try { lift.init(hwMap); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "problem initializing lift");
        }
        try { gimbal.init(hwMap); }
        catch (Exception e) {
            opMode.telemetry.addData("Status", "problem initializing gimbal");
        }
//        try { sensors.init(hwMap); }
//        catch (Exception e) {
//            opMode.telemetry.addData("Status", "problem initializing sensors");
//        }

        hubs = new HashMap<>(2);
        responses = new HashMap<>(2);
        try {
            hubs.put(
                    hwMap.get(DcMotorController.class, "Expansion Hub 1"),
                    hwMap.get(LynxModuleIntf.class, "Expansion Hub 1"));
        } catch (Exception e) {
            opMode.telemetry.addData("Status", "problem with hub1");
        }
        try {
            hubs.put(
                    hwMap.get(DcMotorController.class, "Expansion Hub 2"),
                    hwMap.get(LynxModuleIntf.class, "Expansion Hub 2"));
        } catch (Exception e) {
            opMode.telemetry.addData("Status", "problem with hub2");
        }
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

    public void updateSubsystems() {
        for (LynxModuleIntf hub : hubs.values()) {
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(hub);
            try {
                responses.put(hub, command.sendReceive());
                updated = true;
            } catch (Exception e) {
                Log.e("get bulk data error", e.getMessage());
                updated = false;
            }
        }
//        opMode.telemetry.addData("REV Hubs Updated:", updated);
    }

    /**
     * Waits for opMode's to start. Can perform actions while waiting.
     */
    public void waitForStart() {
        while (!opMode.opModeIsActive() && !opMode.isStopRequested()) {
            opMode.telemetry.addData("status", "waiting for start command...");
            opMode.telemetry.update();
        }
    }

    public void safeSleep(long interval, String msg) {
        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(myTimer.milliseconds() < interval && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Elapsed Time (msec)", "%.00f", myTimer.milliseconds());
            opMode.telemetry.addData("safeSleep", msg);
            opMode.telemetry.update();
            this.opMode.sleep(250);
        }
    }

    public int getEncoderPosition(DcMotor motor) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hubs.get(motor.getController())).getEncoder(motor.getPortNumber()) * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
        }
    }

    public int getMotorVelocity(DcMotor motor) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hubs.get(motor.getController())).getVelocity(motor.getPortNumber());
        }
    }


    /**
     * Autonomous action for landing the robot using the lift mechanism.
     */
    public void land() {
        if (opMode.opModeIsActive()) {
            lift.liftToPos(1, LiftConstants.EXTENDED_POSITION); // 10 inch
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
            drivetrain.turnPID(-RIGHT_TURN, 90);
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
                drivetrain.turnPID(RIGHT_TURN, 90);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE / 2.0, 5);
                drivetrain.turnPID(-RIGHT_TURN,90);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                //turn right to cube
                drivetrain.turnPID(-RIGHT_TURN, 90);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.TILE_HYPOTENUSE / 2.0, 5);
                //turn left
                drivetrain.turnPID(RIGHT_TURN, 90);
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
                drivetrain.turnPID(-DIAGONAL_TURN, 45);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 4);
            } else if (location == TensorFlowManager.TFLocation.CENTER || location == TensorFlowManager.TFLocation.NONE){
                drivetrain.turnPID(-DIAGONAL_TURN, 45);
                opMode.telemetry.addData("Sampling from", "center");
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                drivetrain.turnPID(DIAGONAL_TURN, 45);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 4);
                drivetrain.turnPID(-RIGHT_TURN, 90);
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

            safeSleep(300, "");
            //drive backward 8 inches
            drivetrain.driveToPos(DRIVE_SPEED,-6, 3);
            //turn counter-clockwise face depot
            drivetrain.turnPID(RIGHT_TURN, 90);
            //drive forward one floor tile, then correct position by strafing right a certain number of units
            // then go forward again to depot
            if (location == TensorFlowManager.TFLocation.LEFT){
                //if gold was left, strafe right a floor tile
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE, 4);
                drivetrain.turnPID(DIAGONAL_TURN, 45);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 4.0, 5);
            } else if (location == TensorFlowManager.TFLocation.CENTER || location == TensorFlowManager.TFLocation.NONE){
                //if gold was center or not found, strafe right 1.5 floor tile
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 1.5, 5);
                drivetrain.turnPID(DIAGONAL_TURN, 45);
                drivetrain.driveToPos(DRIVE_SPEED,-FieldConstants.FLOOR_TILE * 4.0, 5);
            } else if (location == TensorFlowManager.TFLocation.RIGHT){
                //if gold was right, strafe right 2 floor tiles
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 2, 4);
                drivetrain.turnPID(DIAGONAL_TURN, 45);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 4.0, 5);
            }
        }
    }

    public void dropMarker() {
        if (opMode.opModeIsActive()) { }
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
                drivetrain.turnPID(-180, 180);
                drivetrain.driveToPos(DRIVE_SPEED, -FieldConstants.FLOOR_TILE * 3.5, 5);
            } else {
                drivetrain.driveToPos(DRIVE_SPEED, FieldConstants.FLOOR_TILE * 3.5, 5);
            }
        }
    }
}
